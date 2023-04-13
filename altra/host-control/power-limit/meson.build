inc_dirs = [include_directories('../../../'),
            ]

executable(
    'ampere-soc-power', 'ampere-soc-power.cpp',
    dependencies: [
        dependency('systemd'),
        dependency('sdbusplus'),
        dependency('threads'),
    ],
    install: true,
    install_dir: get_option('bindir'),
    include_directories : inc_dirs,
)

systemd = dependency('systemd')

configure_file(
  input: 'xyz.openbmc_project.AmpSocPower.service',
  output: 'xyz.openbmc_project.AmpSocPower.service',
  copy: true,
  install_dir: systemd.get_variable('systemdsystemunitdir')
  )
