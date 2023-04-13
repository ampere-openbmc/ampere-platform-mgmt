inc_dirs = [include_directories('./include/'),
            include_directories('../../../'),
            ]

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
        include_directories : inc_dirs,
        )

systemd = dependency('systemd')
systemd_system_unit_dir = systemd.get_variable(
    'systemdsystemunitdir',
    pkgconfig_define: ['prefix', get_option('prefix')])

configure_file(
    copy: true,
    input: 'xyz.openbmc_project.ampere_host_error_monitor.service',
    install: true,
    install_dir: systemd_system_unit_dir,
    output: 'xyz.openbmc_project.ampere_host_error_monitor.service'
)
