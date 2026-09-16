"""Security of the UI node's server."""

import datetime
import ipaddress
import os
import socket
import ssl
import struct
from pathlib import Path
from typing import List, Tuple

from attrs import define

# Where the state (certificate, keys, session secret) is kept, if not the default
DATA_DIR_ENV = "SUGARCOAT_UI_DATA_DIR"
# A certificate and key handed over by whoever runs the recipe
TLS_CERT_ENV = "SUGARCOAT_UI_TLS_CERT"
TLS_KEY_ENV = "SUGARCOAT_UI_TLS_KEY"

CERT_VALIDITY = datetime.timedelta(days=730)
RENEW_BEFORE = datetime.timedelta(days=30)

# TLS 1.2 ciphers with forward secrecy and authenticated encryption
TLS_CIPHERS = "ECDHE+AESGCM:ECDHE+CHACHA20"
TLS_MIN_VERSION = ssl.TLSVersion.TLSv1_2

_MINTED_CERT = "tls.crt"
_MINTED_KEY = "tls.key"


class CertificateError(RuntimeError):
    """A TLS certificate could not be found, read or minted"""


@define(frozen=True)
class Certificate:
    """The certificate the UI serves, and where it came from"""

    certificate: Path
    key: Path
    # "environment", "minted" (on start) or "stored" (minted earlier)
    source: str
    fingerprint: str
    expires: datetime.datetime
    # Addresses the certificate is valid for
    addresses: Tuple[str, ...]


def state_dir() -> Path:
    """The UI's state directory, created with owner-only access if missing"""
    configured = os.environ.get(DATA_DIR_ENV, "").strip()
    path = (
        Path(configured).expanduser()
        if configured
        else Path.home() / ".local" / "share" / "sugarcoat" / "ui"
    )
    path.mkdir(parents=True, exist_ok=True)
    path.chmod(0o700)
    return path


def resolve_certificate() -> Certificate:
    """The certificate to serve, from the first source that provides one.

    1. `SUGARCOAT_UI_TLS_CERT` and `SUGARCOAT_UI_TLS_KEY`
    2. Sugarcoat's own, minted in the state directory and renewed near expiry

    :raises CertificateError: If a configured certificate cannot be used, or none
        can be minted
    """
    env_cert = os.environ.get(TLS_CERT_ENV, "").strip()
    env_key = os.environ.get(TLS_KEY_ENV, "").strip()
    if env_cert or env_key:
        if not (env_cert and env_key):
            raise CertificateError(
                f"{TLS_CERT_ENV} and {TLS_KEY_ENV} must be set together"
            )
        return load_certificate(Path(env_cert), Path(env_key), "environment")

    return minted_certificate(state_dir())


def minted_certificate(directory: Path) -> Certificate:
    """Sugarcoat's own certificate in `directory`, minted when missing, unusable
    or within `RENEW_BEFORE` of expiry"""
    directory.mkdir(parents=True, exist_ok=True)
    cert_path, key_path = directory / _MINTED_CERT, directory / _MINTED_KEY
    if cert_path.is_file() and key_path.is_file():
        try:
            current = load_certificate(cert_path, key_path, "stored")
        except CertificateError:
            current = None
        now = datetime.datetime.now(datetime.timezone.utc)
        if current is not None and current.expires - now > RENEW_BEFORE:
            return current
    _mint(cert_path, key_path)
    return load_certificate(cert_path, key_path, "minted")


def banner(certificate: Certificate) -> List[str]:
    """UI print at startup about the certificate"""
    lines = []
    if certificate.source == "minted":
        lines += [
            f"Minted a TLS certificate: {certificate.certificate}",
            f"Fingerprint (SHA-256): {certificate.fingerprint}",
            f"Expires: {certificate.expires:%Y-%m-%d}",
        ]
    missing = uncovered_addresses(certificate)
    if missing:
        lines.append(
            f"The certificate does not cover this machine's addresses {missing}. "
            "Clients that check host names should connect by a covered name or pin the fingerprint"
        )
    return lines


def uncovered_addresses(certificate: Certificate) -> List[str]:
    """This machine's IPv4 addresses the certificate does not cover"""
    covered = set(certificate.addresses)
    return [address for address in local_ipv4_addresses() if address not in covered]


def local_ipv4_addresses() -> List[str]:
    """IPv4 addresses of this machine's network interfaces (Linux)"""
    import fcntl

    siocgifaddr = 0x8915
    addresses = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _, name in socket.if_nameindex():
            request = struct.pack("256s", name[:15].encode())
            try:
                reply = fcntl.ioctl(sock.fileno(), siocgifaddr, request)
            except OSError:
                continue  # the interface has no IPv4 address
            addresses.append(socket.inet_ntoa(reply[20:24]))
    return addresses


def load_certificate(cert_path: Path, key_path: Path, source: str) -> Certificate:
    """Read a certificate and check that its key belongs to it"""
    from cryptography import x509
    from cryptography.hazmat.primitives import hashes

    try:
        cert = x509.load_pem_x509_certificate(cert_path.read_bytes())
    except (OSError, ValueError) as e:
        raise CertificateError(f"Cannot read the certificate '{cert_path}': {e}") from e
    try:
        ssl.create_default_context(ssl.Purpose.CLIENT_AUTH).load_cert_chain(
            str(cert_path), str(key_path)
        )
    except (OSError, ssl.SSLError) as e:
        raise CertificateError(
            f"The key '{key_path}' cannot be used with the certificate "
            f"'{cert_path}': {e}"
        ) from e

    # `not_valid_after_utc` came only after cryptography 42
    expires = getattr(cert, "not_valid_after_utc", None)
    if expires is None:
        expires = cert.not_valid_after.replace(tzinfo=datetime.timezone.utc)
    try:
        names = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName)
        addresses = tuple(str(a) for a in names.value.get_values_for_type(x509.IPAddress))
    except x509.ExtensionNotFound:
        addresses = ()
    fingerprint = ":".join(f"{b:02X}" for b in cert.fingerprint(hashes.SHA256()))
    return Certificate(cert_path, key_path, source, fingerprint, expires, addresses)


def _mint(
    cert_path: Path, key_path: Path, validity: datetime.timedelta = CERT_VALIDITY
) -> None:
    """Mint a self-signed ECDSA P-256 certificate for this machine"""
    try:
        from cryptography import x509
        from cryptography.hazmat.primitives import hashes, serialization
        from cryptography.hazmat.primitives.asymmetric import ec
        from cryptography.x509.oid import ExtendedKeyUsageOID, NameOID
    except ImportError as e:
        raise CertificateError(
            "Minting a certificate needs the 'cryptography' package "
            "(python3-cryptography)"
        ) from e

    hostname = socket.gethostname()
    dns_names = sorted({"localhost", hostname, f"{hostname}.local"})
    addresses = ["127.0.0.1", "::1"] + [
        a for a in local_ipv4_addresses() if a != "127.0.0.1"
    ]
    key = ec.generate_private_key(ec.SECP256R1())
    name = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, hostname)])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        # A little slack for clients whose clock is slightly behind
        .not_valid_before(now - datetime.timedelta(minutes=5))
        .not_valid_after(now + validity)
        .add_extension(
            x509.SubjectAlternativeName(
                [x509.DNSName(n) for n in dns_names]
                + [x509.IPAddress(ipaddress.ip_address(a)) for a in addresses]
            ),
            critical=False,
        )
        .add_extension(x509.BasicConstraints(ca=False, path_length=None), critical=True)
        .add_extension(
            x509.KeyUsage(
                digital_signature=True,
                content_commitment=False,
                key_encipherment=False,
                data_encipherment=False,
                key_agreement=False,
                key_cert_sign=False,
                crl_sign=False,
                encipher_only=False,
                decipher_only=False,
            ),
            critical=True,
        )
        .add_extension(
            x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]), critical=False
        )
        .sign(key, hashes.SHA256())
    )

    try:
        _write(
            key_path,
            key.private_bytes(
                serialization.Encoding.PEM,
                serialization.PrivateFormat.PKCS8,
                serialization.NoEncryption(),
            ),
            mode=0o600,
        )
        _write(cert_path, cert.public_bytes(serialization.Encoding.PEM), mode=0o644)
    except OSError as e:
        raise CertificateError(f"Cannot write the certificate in '{cert_path.parent}': {e}") from e


def _write(path: Path, data: bytes, mode: int) -> None:
    """Write a file atomically, created with `mode` so a key is never readable by others"""
    temporary = path.with_name(path.name + ".tmp")
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    with os.fdopen(fd, "wb") as f:
        f.write(data)
    os.chmod(temporary, mode)
    os.replace(temporary, path)
