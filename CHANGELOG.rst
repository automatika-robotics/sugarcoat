^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package automatika_ros_sugar
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2025-06-28)
------------------
* (refactor) Corrects type hint in callbacks
* (refactor) Adds type hints to validators
* (refactor) Minor improvements and typo correction
* (refactor) Resolves todo in component actions for active flag and adds default logger for module
* (chore) Updates service creation script with better error handling
* (docs) Updates installation instructions
* (refactor) Removes numpy-quaternion from dependencies and implements rotations using numpy
* (docs) Updates logo icon and adds 'config from file' page to docs
* (fix) Adds wait for node activation after restart and fixes optional arguments parsing in Component
* (docs) Updates events docs
* (docs) Adds international readmes
* (chore) Removes pip based test dependencies for ROS build farm
* (docs) Updates readme
* Contributors: ahr, mkabtoul

0.3.0 (2025-06-18)
------------------
* (chore) Updates installation instructions and CI
* (fix) Removes NoneType from types for compatibility with python < 3.10
* (fix) Add missing pyyaml dependency
* (feature) Adds config parsing from toml, json and yaml and removes Omegaconf dependency
* (fix) Add missing action decorator to methods
* (docs) Updates img names
* (docs) Updates readme
* (chore) Adds a sweeter name for ROS Sugar .. Sugarcoat
* (docs) Minor edits
* (docs) Adds documentation for creating systemd services with scripts
* (docs) Fixes size of logo
* (docs) Fixes warnings in docs
* (docs) Minor fix
* (docs) Updates supported types docs
* (fix) Fixes minor typo
* (fix) Fixes logging level parsing
* (feature) Adds node logging level and rclpy logging level to component config
* (fix) Adds BGR2RGB tranformation for yuv422_yuy2 encoding
* (docs) Removes unused extension
* (docs) Adds docs for components services and pkg advantages
* Merge branch 'feature/system_management' of github.com:automatika-robotics/ros-sugar into feature/system_management
* (fix) Fixes transformation quaternion parsing
* (feature) Adds utility method for processing all ROS image encodings
* (feature) Adds transformation/rotation propertires to TFListener
* (fix) Fixes lifecycle transitioin callback in component
* (chore) Adds msgpack-numpy and attrs as deb package dependencies
* (fix) Handles lifecycle actions start/stop/restart from the launcher
* (fix) Adds action goal lock to handle aborting ongoing goals and executing new incoming goals
* Contributors: ahr, mkabtoul

0.2.9 (2025-02-18)
------------------
* (docs) Updates supported types docs and adds docstrings
* (feature) Adds a script to  make any python script a systemd service
* (fix) Minor fix to check for action server creation before destruction
* (fix) Adds algorithm config from yaml if available
* (fix) Removes setproctitle as a hard dependency
* (fix) Checks for subscription in got_inputs method
* (fix) Fixes type hints for python3.8 compatibility
* Contributors: ahr, mkabtoul

0.2.8 (2025-01-28)
------------------
* (fix) Removes testing to keep build stable until dependencies are merged in rosdistro
* Contributors: ahr

0.2.7 (2025-01-28)
------------------
* (fix) Fixes ChildComponent class in tests
* (fix) Minor fix in component execution_step
* (feature) Adds pytests to package.xml and CMakeLists
* (feature) Adds tests for Actios and ComponentRunType
* (fix) Fixes health_status publishing for non timed components
* (feature) Adds pytest for all event classes and fixes existing errors
* (feature) Adds support for std_msgs MultiArray messages
* (fix) Fixes Event trigger value check and OnGreater event serialization
* Contributors: Maria Kabtoul, mkabtoul

0.2.6 (2025-01-17)
------------------
* (fix) Fixes type hint
* (fix) Fixes getting available events
* (feature) checks for components and events duplicate names
* (fix) Changes type of monitor components to activate
* (chore) Fixes OS versions in CI
* (chore) Adds arms builds to debian packaging
* (refactor) Changes the fuction to create events from jsons
* (fix) Fixes events parsing using serialized events as dictionary keys
* (docs) Adds verification tag
* (docs) Adds external links to docs
* (docs) Adds source link to docs
* Contributors: ahr, mkabtoul

0.2.5 (2025-01-07)
------------------
* (fix) Gets imports and default values based on installed distro
* (fix) Fix launch and launch_ros imports based on ros distro
* Contributors: ahr, mkabtoul

0.2.4 (2024-12-27)
------------------
* (fix) Adds algorithm auto re-configuration from YAML file
* (fix) Fixes callback got_msg property
* (feature) Adds topics callbacks/conversions reparsing to component
  Supports running components from different packages in one script and each component uses its own package callbacks/conversions
* (fix) Updates AllowedTopics config and its validator
* (refactor) Removes PIL as a dependancy
* (fix) Fixes component state transition logging
* (fix) Fixes order to custom method execution in component lifecycle transition methods
* (refactor) Removes BaseNode class
* (fix) Fixes packaging workflow formatting
* (fix) Removes redundant methods from components
* (chore) Increments release action version
* (chore) Adds new action in debs creation workflow
* (refactor) Formats utils
* (refactor) Minor refactoring in utils
* (fix) Removes fix for color correction as the transformation is now applied at the time of visualization
* (fix) Adds color transformation when reading images of yuv encoding
* (chore) Changes name of release action
* (feature) Adds component algorithm config management to the api
* (fix) fixes datatypes update method for using multiple packages
* (chore) Cleans up cmake and packaging
* (refactor) Improves error message when a topic of unsupported type is created
* (refactor) Handles additional datatypes provided by user packages
* (fix) Pins release mirror workflow to run only on release publishing
* (fix) Adds branch name to release workflow
* (fix) Fixes name of action
* (feature) Adds release mirror action
* (docs) Removes autogenerated docs
* (docs) Adds minor modification to readme
* (docs) Changes package description
* (feature) Adds ExecuteMethod service to BaseComponent
* (fix) Minor fix in conversion method
* (refactor) Makes compressed image a realization of image
* (fix) Fixes ros compressed image conversion util
* (feature) Adds support for CompressedImage msg
* (feature) Adds ros_log_level option to each added package
* (feature) Adds additional supported types argument to BaseComponent and Topic validators
* (fix) Adds algorithm auto re-configuration from YAML file
* (fix) Fixes callback got_msg property
* (feature) Adds topics callbacks/conversions reparsing to component
  Supports running components from different packages in one script and each component uses its own package callbacks/conversions
* (fix) Updates AllowedTopics config and its validator
* (refactor) Removes PIL as a dependancy
* (fix) Fixes component state transition logging
* (fix) Fixes order to custom method execution in component lifecycle transition methods
* (refactor) Removes BaseNode class
* (fix) Fixes packaging workflow formatting
* (fix) Removes redundant methods from components
* (chore) Increments release action version
* (chore) Adds new action in debs creation workflow
* (refactor) Formats utils
* (refactor) Minor refactoring in utils
* (fix) Removes fix for color correction as the transformation is now applied at the time of visualization
* (fix) Adds color transformation when reading images of yuv encoding
* (chore) Changes name of release action
* (feature) Adds component algorithm config management to the api
* (fix) fixes datatypes update method for using multiple packages
* (chore) Cleans up cmake and packaging
* (refactor) Improves error message when a topic of unsupported type is created
* (refactor) Handles additional datatypes provided by user packages
* (fix) Pins release mirror workflow to run only on release publishing
* (fix) Adds branch name to release workflow
* (fix) Fixes name of action
* (feature) Adds release mirror action
* (docs) Removes autogenerated docs
* (docs) Adds minor modification to readme
* (docs) Changes package description
* (feature) Adds ExecuteMethod service to BaseComponent
* (fix) Fixes OccupnacyGrid data publishing from numpy
* (fix) Minor fix in conversion method
* (refactor) Makes compressed image a realization of image
* (fix) Fixes ros compressed image conversion util
* (feature) Adds support for CompressedImage msg
* (feature) Adds ros_log_level option to each added package
* (feature) Adds additional supported types argument to BaseComponent and Topic validators
* (fix) Merge pull request `#14 <https://github.com/automatika-robotics/ros-sugar/issues/14>`_
* (chore) Updates package name to automatika_ros_sugar
* (fix) Checks numpy array shape in OccupancyGrid converter
* (feature) Adds stamped header and frame_id to ros publishers/callbacks
* (docs) Updates install instructions
* Contributors: ahr, mkabtoul

0.2.3 (2024-11-13)
------------------
* (chore) bump version 0.2.2 -> 0.2.3
* (chore) Adds deb packaging scripts and actions (`#13 <https://github.com/automatika-robotics/ros-sugar/issues/13>`_)
* (docs) Removes notice
* Contributors: ahr

0.2.2 (2024-11-04)
------------------
* (chore) bump version 0.2.1 -> 0.2.2
* (feature) Adds activation timeout to monitor and launcher
* (fix) Fixes publishing numpy data to ROS OcuupancyGrid
* (refactor) Updates OccupancyGrid get_output using numpy operations
* Contributors: mkabtoul

0.2.1 (2024-10-29)
------------------
* (chore) bump version 0.2.0 -> 0.2.1
* (feature) Adds support for external tool calling in multiprocessing
* Contributors: ahr

0.2.0 (2024-10-25)
------------------
* (chore) Bump version 0.1.1 -> 0.2.0
* Merge pull request `#12 <https://github.com/automatika-robotics/ros-sugar/issues/12>`_ from automatika-robotics/feature/external_processors
  Adds external processor support when running components in multiprocessing
* (refactor) Makes msgpack a global dependancy
* (fix) Fixes deserialization of external processors and handling of processor result in launcher
* (fix) Corrects the serialization of numpy arrays within lists
* (feature) Changes defaults for launcher parameters when using multiprocessing
* (fix) Fixes handling composite type check for deserialization and input/output deserialization in components
* (fix) Adds node name as parameter to callbacks for init
* (fix) Adds alias to attrs private attribute in BaseComponentConfig
* (fix) Restores executable to old version
* Merge branch 'feature/external_processors' of github.com:automatika-robotics/ros-sugar into feature/external_processors
* (fix) Fixes new method name in launcher
* (fix) Moves callbackgroup to BaseComponentConfig and changes initialization of inputs/outputs in component
* (fix) Fixes serialization of callbackgroup in config
* (fix) Fixes type hints for compatibility
* (docs) Fixes ubuntu version for dependancy problems
* (refactor) Makes msgpack a functional dependency
* (refactor) Adds handling of callback group and input/output initialization to facilitate multiprocessing
* (feature) Adds handling of callback group for multiprocess launch
* (fix) Adds serialization of np arrays and tuples
* (fix) Adds converter for QoS profile for serialization
* (refactor) Changes inputs/outputs handling in executable
* (refactor) Changes name of enum convert utility function
* (fix) Fixes use of multi processors for same topic in launcher
* (fix) Fix package installation for documentation workflow
* (feature) Adds support for multiple external processors on the same topic
* (fix) Fixes visibility of external_processors to protected
* (fix) Fixes typo in attaching external preprocessors
* (feature) Adds unix socket based listener threads for using external processors with components being run in multiprocessing
  - Modifies executable to add an argument for external processors
  - Adds setting and getting for external processor json in component
  - Adds setting up of external processors on component activation and destruction on component stop
  - Adds setup of external processor sockets and thread pool in launcher
* (fix) Moves callbackgroup to BaseComponentConfig and changes initialization of inputs/outputs in component
* (fix) Fixes serialization of callbackgroup in config
* (fix) Fixes type hints for compatibility
* (docs) Fixes ubuntu version for dependancy problems
* (refactor) Makes msgpack a functional dependency
* (refactor) Adds handling of callback group and input/output initialization to facilitate multiprocessing
* (feature) Adds handling of callback group for multiprocess launch
* (fix) Adds serialization of np arrays and tuples
* (fix) Adds converter for QoS profile for serialization
* (refactor) Changes inputs/outputs handling in executable
* (refactor) Changes name of enum convert utility function
* (feature) Adds event processing options and supports lists in event values
  Adds options to handle an event once or handle with a time delay
* (fix) Uses List from typing in type hints
* (feature) Adds handle_once and event_delay options to Event
* (feature) Adds list to supported event trigger values
* (fix) Handles keep_alive in component parameter update service requests
* (fix) Passes monitor executor to service client send_req
* (fix) Fixes use of multi processors for same topic in launcher
* (fix) Fix package installation for documentation workflow
* (feature) Adds support for multiple external processors on the same topic
* (fix) Fixes visibility of external_processors to protected
* (fix) Fixes typo in attaching external preprocessors
* (feature) Adds unix socket based listener threads for using external processors with components being run in multiprocessing
  - Modifies executable to add an argument for external processors
  - Adds setting and getting for external processor json in component
  - Adds setting up of external processors on component activation and destruction on component stop
  - Adds setup of external processor sockets and thread pool in launcher
* (fix) Fixes minor bugs in base component and launcher (`#10 <https://github.com/automatika-robotics/ros-sugar/issues/10>`_)
* (fix) Fixes the handling of yuv422_yuy2 encoding in image reading util function
* (fix) Adds process id to monitor node name
* (fix) Fixes type check for callables in attaching post and pre processors
* (fix) Updates component launch arguments after parsing events_actions
* (docs) Updates docs url links in readme
* (docs) Adds github workflow for docs (`#9 <https://github.com/automatika-robotics/ros-sugar/issues/9>`_)
* (fix) Adds handling image encodings with alpha channel
* Create LICENSE
* Initial release version 0.1.1 (`#8 <https://github.com/automatika-robotics/ros-sugar/issues/8>`_)
* init commit
* Contributors: ahr, aleph-ra, mkabtoul
