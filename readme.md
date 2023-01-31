to build
```
cd AI_deck_send_vel_v2
docker run --rm -v ${PWD}:/module aideck-with-autotiler tools/build/make-example . image
```
to flash

```
cfloader flash ./BUILD/GAP8_V2/GCC_RISCV_FREERTOS/target.board.devices.flash.img deck-bcAI:gap8-fw -w radio://0/80/2M
```