# Images for display library

1. Create image with size fitting screen
2. Mirror image horizontally
3. Rotate 90 degree counter-clockwise
4. Invert image colors to negative
5. Convert to XBM via https://www.online-utility.org/image/convert/to/XBM 
6. Change ending from `*.XBM` to `*.h`
7. Swap width and height definiton values
8. Replace `static char 1661509677887_bits[] = {` with `const uint8_t boot_logo_bits[] PROGMEM = {`