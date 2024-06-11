int data_length = 0;
uint8_t* data;

void close_strip()
{
    free(data);
}

uint8_t* initialize_strip(int leds_count)
{
    data_length = leds_count * 3;
    data = (uint8_t*) malloc(leds_count * 3);
    return data;
}

void render_strip() {
    printf("Rendering:");
    for (int i = 0; i < data_length; ++i) {
        printf(" 0x%02x", data[i]);
    }
    printf("\n");
}
