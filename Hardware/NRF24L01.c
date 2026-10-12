#include "NRF24L01.h"

#define NRF_R_REGISTER 0x00
#define NRF_W_REGISTER 0x20
#define NRF_R_RX_PAYLOAD 0x61
#define NRF_FLUSH_TX 0xE1
#define NRF_FLUSH_RX 0xE2
#define NRF_NOP 0xFF
#define NRF_CONFIG 0x00
#define NRF_EN_AA 0x01
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_AW 0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_RX_ADDR_P0 0x0A
#define NRF_RX_PW_P0 0x11

#define NRF_CE_PIN GPIO_Pin_8
#define NRF_CSN_PIN GPIO_Pin_15
#define NRF_SCK_PIN GPIO_Pin_3
#define NRF_MISO_PIN GPIO_Pin_4
#define NRF_MOSI_PIN GPIO_Pin_5

uint8_t NRF24L01_RxPacket[4];
static const uint8_t nrf_address[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

static void nrf_ce(uint8_t v) { GPIO_WriteBit(GPIOA, NRF_CE_PIN, (BitAction)v); }
static void nrf_csn(uint8_t v) { GPIO_WriteBit(GPIOA, NRF_CSN_PIN, (BitAction)v); }

static uint8_t nrf_swap(uint8_t value)
{
    uint8_t i;
    for (i = 0; i < 8; ++i) {
        GPIO_WriteBit(GPIOB, NRF_MOSI_PIN, (value & 0x80) ? Bit_SET : Bit_RESET);
        value <<= 1;
        GPIO_SetBits(GPIOB, NRF_SCK_PIN);
        if (GPIO_ReadInputDataBit(GPIOB, NRF_MISO_PIN)) value |= 1;
        GPIO_ResetBits(GPIOB, NRF_SCK_PIN);
    }
    return value;
}

static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t value;
    nrf_csn(0); nrf_swap(NRF_R_REGISTER | reg); value = nrf_swap(NRF_NOP); nrf_csn(1);
    return value;
}

static void nrf_write_reg(uint8_t reg, uint8_t value)
{
    nrf_csn(0); nrf_swap(NRF_W_REGISTER | reg); nrf_swap(value); nrf_csn(1);
}

static void nrf_write_regs(uint8_t reg, const uint8_t *data, uint8_t count)
{
    uint8_t i;
    nrf_csn(0); nrf_swap(NRF_W_REGISTER | reg);
    for (i = 0; i < count; ++i) nrf_swap(data[i]);
    nrf_csn(1);
}

static uint8_t nrf_status(void)
{
    uint8_t value;
    nrf_csn(0); value = nrf_swap(NRF_NOP); nrf_csn(1);
    return value;
}

static void nrf_receive_mode(void)
{
    uint8_t config = nrf_read_reg(NRF_CONFIG);
    nrf_ce(0); nrf_write_reg(NRF_CONFIG, config | 0x03); nrf_ce(1);
}

void NRF24L01_Init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    gpio.GPIO_Mode = GPIO_Mode_Out_PP; gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = NRF_CE_PIN | NRF_CSN_PIN; GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = NRF_SCK_PIN | NRF_MOSI_PIN; GPIO_Init(GPIOB, &gpio);
    gpio.GPIO_Mode = GPIO_Mode_IPU; gpio.GPIO_Pin = NRF_MISO_PIN; GPIO_Init(GPIOB, &gpio);
    nrf_ce(0); nrf_csn(1); GPIO_ResetBits(GPIOB, NRF_SCK_PIN | NRF_MOSI_PIN);
    nrf_write_reg(NRF_CONFIG, 0x08); nrf_write_reg(NRF_EN_AA, 0x3F);
    nrf_write_reg(NRF_EN_RXADDR, 0x01); nrf_write_reg(NRF_SETUP_AW, 0x03);
    nrf_write_reg(NRF_SETUP_RETR, 0x03); nrf_write_reg(NRF_RF_CH, 0x02);
    nrf_write_reg(NRF_RF_SETUP, 0x0E); nrf_write_reg(NRF_RX_PW_P0, sizeof(NRF24L01_RxPacket));
    nrf_write_regs(NRF_RX_ADDR_P0, nrf_address, sizeof(nrf_address));
    nrf_csn(0); nrf_swap(NRF_FLUSH_TX); nrf_csn(1);
    nrf_csn(0); nrf_swap(NRF_FLUSH_RX); nrf_csn(1);
    nrf_write_reg(NRF_STATUS, 0x70); nrf_receive_mode();
}

uint8_t NRF24L01_Receive(void)
{
    uint8_t i, status = nrf_status();
    if ((nrf_read_reg(NRF_CONFIG) & 0x02) == 0 || (status & 0x30) == 0x30) { NRF24L01_Init(); return 0; }
    if ((status & 0x40) == 0) return 0;
    nrf_csn(0); nrf_swap(NRF_R_RX_PAYLOAD);
    for (i = 0; i < sizeof(NRF24L01_RxPacket); ++i) NRF24L01_RxPacket[i] = nrf_swap(NRF_NOP);
    nrf_csn(1); nrf_write_reg(NRF_STATUS, 0x40);
    nrf_csn(0); nrf_swap(NRF_FLUSH_RX); nrf_csn(1);
    return 1;
}
