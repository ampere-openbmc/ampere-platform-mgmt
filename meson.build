project(
    'ampere-platform-mgmt',
    'cpp',
    default_options: [
        'warning_level=3',
        'werror=true',
        'cpp_std=c++17'
    ],
    license: 'Apache-2.0',
    version: '1.0',
)

# Option for Altra host processor
if get_option('host').contains('altra')
    if get_option('boot-progress').enabled()
        subdir('altra/host-monitor/boot-progress')
    endif

    if get_option('error-monitor').enabled()
        subdir('altra/host-monitor/error-monitor')
    endif

    if get_option('temp-event-log').enabled()
        subdir('altra/host-monitor/temp-event-log')
    endif
endif

