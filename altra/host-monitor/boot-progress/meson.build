executable(
    'ampere-boot-progress', 'bootprogress.cpp',
    dependencies: [
        dependency('systemd'),
        dependency('sdbusplus'),
        dependency('threads'),
    ],
    install: true,
    install_dir: get_option('bindir')
)

systemd = dependency('systemd')

configure_file(
  input: 'xyz.openbmc_project.bootprogress.service',
  output: 'xyz.openbmc_project.bootprogress.service',
  copy: true,
  install_dir: systemd.get_pkgconfig_variable('systemdsystemunitdir')
  )
