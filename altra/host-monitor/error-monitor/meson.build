inc_dir = include_directories('./include/')

deps = [dependency('libsystemd', version : '>=221'),
        dependency('systemd'),
        dependency('sdbusplus'),
        dependency('threads'),
        ]

executable(
        'ampere-host-error-monitor',
        'ampere-host-error-monitor.cpp',
        dependencies: deps,
        install: true,
        include_directories : inc_dir,
        )

systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_pkgconfig_variable(
    'systemdsystemunitdir',
    define_variable: ['prefix', get_option('prefix')])

configure_file(
    copy: true,
    input: 'xyz.openbmc_project.ampere_host_error_monitor.service',
    install: true,
    install_dir: systemd_system_unit_dir,
    output: 'xyz.openbmc_project.ampere_host_error_monitor.service'
)
