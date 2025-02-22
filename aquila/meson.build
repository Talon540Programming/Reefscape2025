project('aquila', ['cpp', 'c'],
        version : '1.0.0',
        default_options : ['warning_level=3', 'cpp_std=c++20'])

cmake = import('cmake')

allwpilib_opts = cmake.subproject_options()
allwpilib_opts.add_cmake_defines({'BUILD_SHARED_LIBS' : 'ON', 'WITH_CSCORE' : 'OFF', 'WITH_EXAMPLES' : 'OFF', 'WITH_GUI' : 'OFF', 'WITH_JAVA' : 'OFF', 'WITH_NTCORE' : 'ON', 'WITH_SIMULATION_MODULES' : 'OFF', 'WITH_TESTS' : 'OFF', 'WITH_WPILIB' : 'OFF', 'WITH_WPIMATH' : 'ON', 'WITH_PROTOBUF' : 'ON', 'WITH_WPIUNITS' : 'OFF' })

allwpilib_subproj = cmake.subproject('wpilib', options: allwpilib_opts)
ntcore = allwpilib_subproj.dependency('ntcore')
wpimath = allwpilib_subproj.dependency('wpimath')

aquila = executable('aquila', 'src/main.cpp', dependencies : [ntcore, wpimath], install : true)
