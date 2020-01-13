# Non-recursive Meson build for LabvCNC unit tests
#
# Individual subdirectories define files / include directories for use in the top-level build
# Avoid defining targets / libraries / dependencies in subdirectories if possible.
# It's a big pain to add an external dependency in a dubdirectory, because it
# needs either kludgy relative paths, or a specific execution order of the
# subdir commands (fragile and hard to maintain / inspect).
#
# Keeping with Meson's general philosophy, the preferred way is to define paths
# for sources and includes in the subdirectories, and the rest in the top-level
# file. This way, the entire dependency tree is present in one file.
# Furthermore, a user needing to add a new source file only has to touch the
# subdirectory file, and it's very obvious what needs to change (no build rule
# clutter there).
#
# General pattern:
# 
# meson.build -> project settings, targets, library definitions, etc.
#    
# src/
#    foo/meson.build -> define files / inc for libfoo
#    bar/meson.build -> define files / inc for libbar
#    baz/meson.build -> define files / inc for libbaz
#
# Suffixes used:
#  _srcs: 'files' object containing source files
#  _inc:  'include_directory' object with internal include paths for a given library (not external dependencies)
#  _external_inc: contains miscellaneous include paths for a library
#    (consider this a temporary fix, includes should ideally be handled by
#     dependency objects)
# _dep:  dependency object (encapsulates include / link dependencies for a library)
#
# TODO
# * Provide unit tests for more libraries beyond interp / motion
# * Build labvcnc libraries with correct build options given a configuration
#

project('labvcnc-unit-test', ['c','cpp'])

# TODO make these arguments specific to libraries if we ever replace autotools
add_global_arguments(['-DULAPI','-DUNIT_TEST','-DTP_PEDANTIC_DEBUG','-Wall'], language : 'c')
add_global_arguments(['-DULAPI','-DUNIT_TEST'], language : 'cpp')

# Common external dependencies 
m_dep = meson.get_compiler('c').find_library('m', required : true)
boost_dep = dependency('boost', modules : ['python'])
python2_dep = dependency('python2')

# Define source files and include paths

subdir('src')
subdir('src/lbv/nml_intf')
subdir('src/lbv/rs274ngc')
subdir('src/lbv/sai')
subdir('src/lbv/pythonplugin')
subdir('src/lbv/tp')
subdir('src/lbv/kinematics')
subdir('src/lbv/motion')
subdir('src/hal')
subdir('src/libnml/inifile')
subdir('src/libnml/nml')
subdir('src/libnml/posemath')
subdir('src/rtapi')

subdir('unit_tests/tp')
subdir('unit_tests/interp')

# Global library dependencies
dl_dep = meson.get_compiler('cpp').find_library('dl', required : true)
m_dep = meson.get_compiler('cpp').find_library('m', required : true)

# Source path locations
src_root = 'src'
tp_dir = join_paths(src_root, 'lbv/tp')

# FIXME avoid converting autotools configuration for now, eventually replace with native config

# INI file library

liblabvcncini_inc = [
  inifile_inc,
  config_inc
]

liblabvcncini = shared_library('inifile',
    inifile_srcs,
    include_directories : liblabvcncini_inc,
    dependencies : [ ]
    )

liblabvcncini_dep = declare_dependency(include_directories : liblabvcncini_inc,
    link_with : liblabvcncini)

# Python plugin

libpyplugin = shared_library('pyplugin',
    pythonplugin_srcs,
    include_directories : pythonplugin_inc,
    dependencies : [ boost_dep, python2_dep, liblabvcncini_dep, dl_dep]
    )

libpyplugin_dep = declare_dependency(include_directories : pythonplugin_inc,
    link_with : libpyplugin)

posemath_external_inc = ([
  config_inc,
  rtapi_inc
])


libposemath = shared_library('posemath',
    posemath_srcs,
    include_directories : [posemath_inc, posemath_external_inc],
    dependencies : [ m_dep ]
    )

libposemath_dep = declare_dependency(include_directories : posemath_inc,
    link_with : libposemath)

libposemath_cpp = shared_library('posemath_cpp',
    posemath_cpp_srcs,
    include_directories : posemath_inc,
    dependencies : [ dl_dep, m_dep, libposemath_dep]
    )

libposemath_cpp_dep = declare_dependency(include_directories : posemath_inc,
    link_with : libposemath_cpp)


# TP sub-set for unit testing only
libtp_unit_test = shared_library('tp_unit_test',
    include_directories : [posemath_inc, posemath_external_inc],
    dependencies : [ m_dep ]
    )

libposemath_dep = declare_dependency(include_directories : posemath_inc,
    link_with : libposemath)


unit_test_inc = include_directories([
  'unit_tests',
])

tp_unit_test_inc = [
  config_inc,
  posemath_inc,
  tp_inc,
  kinematics_inc,
  motion_inc,
  rtapi_inc,
  hal_inc,
  lbvpose_inc,
]

# ULAPI

libulapi = static_library('ulapi',
  ulapi_srcs,
  include_directories : [rtapi_inc, config_inc],
)

libulapi_dep = declare_dependency(include_directories : rtapi_inc,
    link_with : libulapi)

# Define static libraries for easier linking with unit tests
liblbvpose = static_library('lbvpose',
  lbvpose_srcs,
  include_directories : [ lbvpose_inc, posemath_inc, rtapi_inc ],
  dependencies : [ libposemath_dep ]
)

liblbvpose_dep = declare_dependency(include_directories : lbvpose_inc,
    link_with : liblbvpose)

liblabvcnchal = library('labvcnchal',
  hal_lib_srcs,
  dependencies : [m_dep, libulapi_dep],
  include_directories : [hal_inc, tp_unit_test_inc])
liblabvcnchal_dep = declare_dependency(include_directories : hal_inc,
    link_with : liblabvcnchal)

libtp = static_library('tp', 
  tp_srcs,
  include_directories : [ tp_inc, motion_inc, kinematics_inc], 
  dependencies : [libposemath_dep, liblbvpose_dep, libulapi_dep, liblabvcnchal_dep]
)
libtp_dep = declare_dependency(include_directories : tp_inc,
    link_with : libtp)

tp_test_files = [
  'test_blendmath',
  ]
foreach n : tp_test_files
  
test(n, executable(n,
  join_paths('unit_tests/tp', n+'.c'),
  dependencies : [m_dep, libposemath_dep, libtp_dep],
  include_directories : [ tp_unit_test_inc, unit_test_inc ],
  ))

endforeach


rs274ngc_external_inc = [
  config_inc,
  lbvpose_inc,
  motion_inc,
  pythonplugin_inc,
  nml_inc,
  posemath_inc,
]

interp_src = join_paths(src_root, 'lbv/rs274ngc')


librs274ngc = shared_module('rs274ngc',
  rs274ngc_srcs,
  dependencies : [
    boost_dep, 
    python2_dep, 
    dl_dep, 
    liblabvcncini_dep, 
    libpyplugin_dep,
    libposemath_cpp_dep
    ],
  include_directories : [tp_unit_test_inc, rs274ngc_inc, rs274ngc_external_inc],
  link_with: [libposemath_cpp])

librs274ngc_dep = declare_dependency(include_directories : rs274ngc_inc,
    link_with : librs274ngc)

libsaicanon = shared_module('saicanon',
  saicanon_srcs,
  include_directories : [sai_inc, rs274ngc_external_inc ],
  dependencies : [boost_dep, python2_dep, dl_dep, liblabvcncini_dep, librs274ngc_dep]
  )

libsaicanon_dep = declare_dependency(include_directories : sai_inc,
    link_with : libsaicanon)

test_interp_ex = executable('test_interp',
    test_interp_srcs,
    include_directories : [test_interp_inc, rs274ngc_external_inc, unit_test_inc],
    dependencies: [
        dl_dep,
        python2_dep,
        librs274ngc_dep,
        libpyplugin_dep,
        liblabvcnchal_dep,
        libsaicanon_dep,
        ]
    )

test('test_interp', test_interp_ex)


