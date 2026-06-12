# IRSend Investigations
## Aim
To figure out how best to rewrite the ESP32 support to use the Remote Control Transceiver ([RMT])

## Questions
1. Can we hook in at `IRsend` class level?
   1. Seems the sernsible place to go for the moment.  This is below, for esample, Manchester encoding implementation.
1. Does it make sense to hook higher than this for common modules that just call `IRsend::send()`
    1. Without modulation we should be able to use `IRsend` methods to build the [RMT] buffer and then use [RMT] to send it.
1. When instatiating `IRsend`, what is the `use_modulation` flag for?  Does anything actually use it?
    1. AI claims that only IRsendTest sets `use_modulation` true either explcitly or implcitly.  Presumably other implementations set it false.
1. If we reimplemented `IRsend` is there a good interface between `IRsend` the the [RMT] that we can define?
    1. AI has had a go at creating one!  But _does it blend_?
1. This comment to force Push actions to test code.

[RMT]: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/rmt.html