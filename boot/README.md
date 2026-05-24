# Boot config fragments

These are excerpts you paste once into `/boot/firmware/config.txt` on the
rover — they aren't full config files. The boot partition isn't tracked
by this repo, so the fragments here are the canonical record of what we
expect to be applied.

## Apply

Append the contents of `config-additions.txt` to `/boot/firmware/config.txt`,
then reboot.

## Verify after reboot

- `vcgencmd get_config arm_freq` → `2000`
- `vcgencmd measure_temp` → under 75 °C at idle
- `vcgencmd get_throttled` → `0x0` during normal driving

## Do not

Raise `arm_freq` above 2000 unless an active cooler is installed on the Pi 5.
Without one, the SoC sits at ~78 °C idle and will throttle (or eventually
shorten its life) under sustained load.
