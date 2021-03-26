inc_dirs = [include_directories('../../../'),
            ]

executable(
    'ampere-scp-failover',
    [
        'scp-failover.cpp',
        'gpio.cpp',
    ],
    dependencies: [
        dependency('systemd'),
        dependency('sdbusplus'),
        dependency('threads'),
        dependency('gpioplus'),
    ],
    install: true,
    install_dir: get_option('bindir'),
    include_directories : inc_dirs,
)

systemd = dependency('systemd')

configure_file(
  input: 'ampere-scp-failover.service',
  output: 'ampere-scp-failover.service',
  copy: true,
  install_dir: systemd.get_pkgconfig_variable('systemdsystemunitdir')
  )