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

packagedir = join_paths(
    get_option('prefix'),
    get_option('datadir'),
    meson.project_name(),
)

configfile = 'config.json'
confpath = '"' + join_paths(
    packagedir,
    configfile,
) + '"'

install_data(sources : configfile, install_dir : packagedir)

conf_data = configuration_data()
conf_data.set('AMPERE_PLATFORM_MGMT_CONFIG_FILE', confpath)

configure_file(output : 'platform_config.hpp',
               configuration : conf_data)

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

    if get_option('power-limit').enabled()
        subdir('altra/host-control/power-limit')
    endif
endif

