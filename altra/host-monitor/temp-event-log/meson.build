executable(
    'tempevent_log', 'tempevent_log.cpp',
    dependencies: [
        dependency('systemd'),
        dependency('sdbusplus'),
        dependency('threads'),
    ],
    install: true,
    install_dir: get_option('bindir')
)
