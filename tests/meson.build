project('tests', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++14'])

add_project_arguments('-DDEBUG', language: 'cpp')

executable('tests',
           'playground.cpp',
           '../src/MazeSolver.cpp',
           install : true)

