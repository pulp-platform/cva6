// File auto-generated by Padrick 0.1.0.post0.dev37+gc5ae078

// Assignment Macros
// Assigns all members of port struct to another struct with same names but potentially different order

`define ASSIGN_PERIPHS_SPI7_PAD2SOC(load, driver) \
  assign load.sd1_i = driver.sd1_o; \

`define ASSIGN_PERIPHS_SPI7_SOC2PAD(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.csn0_i = driver.csn0_o; \
  assign load.sd0_i = driver.sd0_o; \

`define ASSIGN_PERIPHS_CAN2_CAN_PAD2SOC(load, driver) \
  assign load.can1_rx_i = driver.can1_rx_o; \
  assign load.can2_rx_i = driver.can2_rx_o; \

`define ASSIGN_PERIPHS_CAN2_CAN_SOC2PAD(load, driver) \
  assign load.can1_tx_i = driver.can1_tx_o; \
  assign load.can2_tx_i = driver.can2_tx_o; \

`define ASSIGN_PERIPHS_CAM0_PAD2SOC(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.data0_i = driver.data0_o; \
  assign load.data1_i = driver.data1_o; \
  assign load.data2_i = driver.data2_o; \
  assign load.data3_i = driver.data3_o; \
  assign load.data4_i = driver.data4_o; \
  assign load.data5_i = driver.data5_o; \
  assign load.data6_i = driver.data6_o; \
  assign load.data7_i = driver.data7_o; \
  assign load.hsync_i = driver.hsync_o; \
  assign load.vsync_i = driver.vsync_o; \


`define ASSIGN_PERIPHS_SPI8_PAD2SOC(load, driver) \
  assign load.sd1_i = driver.sd1_o; \

`define ASSIGN_PERIPHS_SPI8_SOC2PAD(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.csn0_i = driver.csn0_o; \
  assign load.sd0_i = driver.sd0_o; \

`define ASSIGN_PERIPHS_SPI9_PAD2SOC(load, driver) \
  assign load.sd1_i = driver.sd1_o; \

`define ASSIGN_PERIPHS_SPI9_SOC2PAD(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.csn0_i = driver.csn0_o; \
  assign load.sd0_i = driver.sd0_o; \

`define ASSIGN_PERIPHS_I2C3_PAD2SOC(load, driver) \
  assign load.scl_i = driver.scl_o; \
  assign load.sda_i = driver.sda_o; \

`define ASSIGN_PERIPHS_I2C3_SOC2PAD(load, driver) \
  assign load.scl_i = driver.scl_o; \
  assign load.scl_oe_i = driver.scl_oe_o; \
  assign load.sda_i = driver.sda_o; \
  assign load.sda_oe_i = driver.sda_oe_o; \

`define ASSIGN_PERIPHS_IMU2_DRDY_PAD2SOC(load, driver) \
  assign load.gpio57_i = driver.gpio57_o; \

`define ASSIGN_PERIPHS_IMU2_DRDY_SOC2PAD(load, driver) \
  assign load.gpio57_dir_i = driver.gpio57_dir_o; \
  assign load.gpio57_i = driver.gpio57_o; \

`define ASSIGN_PERIPHS_CAM1_PAD2SOC(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.data0_i = driver.data0_o; \
  assign load.data1_i = driver.data1_o; \
  assign load.data2_i = driver.data2_o; \
  assign load.data3_i = driver.data3_o; \
  assign load.data4_i = driver.data4_o; \
  assign load.data5_i = driver.data5_o; \
  assign load.data6_i = driver.data6_o; \
  assign load.data7_i = driver.data7_o; \
  assign load.hsync_i = driver.hsync_o; \
  assign load.vsync_i = driver.vsync_o; \


`define ASSIGN_PERIPHS_SPI10_PAD2SOC(load, driver) \
  assign load.sd1_i = driver.sd1_o; \

`define ASSIGN_PERIPHS_SPI10_SOC2PAD(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.csn0_i = driver.csn0_o; \
  assign load.sd0_i = driver.sd0_o; \

`define ASSIGN_PERIPHS_I2C4_PAD2SOC(load, driver) \
  assign load.scl_i = driver.scl_o; \
  assign load.sda_i = driver.sda_o; \

`define ASSIGN_PERIPHS_I2C4_SOC2PAD(load, driver) \
  assign load.scl_i = driver.scl_o; \
  assign load.scl_oe_i = driver.scl_oe_o; \
  assign load.sda_i = driver.sda_o; \
  assign load.sda_oe_i = driver.sda_oe_o; \


`define ASSIGN_PERIPHS_SPI10_CS1_SOC2PAD(load, driver) \
  assign load.spi10_cs_i = driver.spi10_cs_o; \

`define ASSIGN_PERIPHS_GPIO58_PAD2SOC(load, driver) \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO58_SOC2PAD(load, driver) \
  assign load.gpio_d_i = driver.gpio_d_o; \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO59_PAD2SOC(load, driver) \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO59_SOC2PAD(load, driver) \
  assign load.gpio_d_i = driver.gpio_d_o; \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO60_PAD2SOC(load, driver) \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO60_SOC2PAD(load, driver) \
  assign load.gpio_d_i = driver.gpio_d_o; \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_SDIO1_PAD2SOC(load, driver) \
  assign load.cmd_i = driver.cmd_o; \
  assign load.data0_i = driver.data0_o; \
  assign load.data1_i = driver.data1_o; \
  assign load.data2_i = driver.data2_o; \
  assign load.data3_i = driver.data3_o; \

`define ASSIGN_PERIPHS_SDIO1_SOC2PAD(load, driver) \
  assign load.clk_i = driver.clk_o; \
  assign load.cmd_i = driver.cmd_o; \
  assign load.cmd_oen_i = driver.cmd_oen_o; \
  assign load.data0_i = driver.data0_o; \
  assign load.data0_oe_i = driver.data0_oe_o; \
  assign load.data1_i = driver.data1_o; \
  assign load.data1_oe_i = driver.data1_oe_o; \
  assign load.data2_i = driver.data2_o; \
  assign load.data2_oe_i = driver.data2_oe_o; \
  assign load.data3_i = driver.data3_o; \
  assign load.data3_oe_i = driver.data3_oe_o; \

`define ASSIGN_PERIPHS_UART5_PAD2SOC(load, driver) \
  assign load.cts_i = driver.cts_o; \
  assign load.rx_i = driver.rx_o; \

`define ASSIGN_PERIPHS_UART5_SOC2PAD(load, driver) \
  assign load.rts_i = driver.rts_o; \
  assign load.tx_i = driver.tx_o; \

`define ASSIGN_PERIPHS_UART6_PAD2SOC(load, driver) \
  assign load.rx_i = driver.rx_o; \

`define ASSIGN_PERIPHS_UART6_SOC2PAD(load, driver) \
  assign load.tx_i = driver.tx_o; \

`define ASSIGN_PERIPHS_GPIO61_PAD2SOC(load, driver) \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO61_SOC2PAD(load, driver) \
  assign load.gpio_d_i = driver.gpio_d_o; \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO62_PAD2SOC(load, driver) \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_GPIO62_SOC2PAD(load, driver) \
  assign load.gpio_d_i = driver.gpio_d_o; \
  assign load.gpio_i = driver.gpio_o; \

`define ASSIGN_PERIPHS_HYPER1_PAD2SOC(load, driver) \
  assign load.dq0_i = driver.dq0_o; \
  assign load.dq1_i = driver.dq1_o; \
  assign load.dq2_i = driver.dq2_o; \
  assign load.dq3_i = driver.dq3_o; \
  assign load.dq4_i = driver.dq4_o; \
  assign load.dq5_i = driver.dq5_o; \
  assign load.dq6_i = driver.dq6_o; \
  assign load.dq7_i = driver.dq7_o; \
  assign load.rwds_i = driver.rwds_o; \

`define ASSIGN_PERIPHS_HYPER1_SOC2PAD(load, driver) \
  assign load.ck_i = driver.ck_o; \
  assign load.ckn_i = driver.ckn_o; \
  assign load.cs0n_i = driver.cs0n_o; \
  assign load.cs1n_i = driver.cs1n_o; \
  assign load.dq0_i = driver.dq0_o; \
  assign load.dq1_i = driver.dq1_o; \
  assign load.dq2_i = driver.dq2_o; \
  assign load.dq3_i = driver.dq3_o; \
  assign load.dq4_i = driver.dq4_o; \
  assign load.dq5_i = driver.dq5_o; \
  assign load.dq6_i = driver.dq6_o; \
  assign load.dq7_i = driver.dq7_o; \
  assign load.dq__i = driver.dq__o; \
  assign load.resetn_i = driver.resetn_o; \
  assign load.rwds_i = driver.rwds_o; \
  assign load.rwds_oe_i = driver.rwds_oe_o; \

`define ASSIGN_PERIPHS_ETH0_PAD2SOC(load, driver) \
  assign load.eth_rxcd_i = driver.eth_rxcd_o; \
  assign load.eth_rxctl_i = driver.eth_rxctl_o; \
  assign load.eth_rxd0_i = driver.eth_rxd0_o; \
  assign load.eth_rxd1_i = driver.eth_rxd1_o; \
  assign load.eth_rxd2_i = driver.eth_rxd2_o; \
  assign load.eth_rxd3_i = driver.eth_rxd3_o; \

`define ASSIGN_PERIPHS_ETH0_SOC2PAD(load, driver) \
  assign load.eth_rst_i = driver.eth_rst_o; \
  assign load.eth_txcd_i = driver.eth_txcd_o; \
  assign load.eth_txctl_i = driver.eth_txctl_o; \
  assign load.eth_txd0_i = driver.eth_txd0_o; \
  assign load.eth_txd1_i = driver.eth_txd1_o; \
  assign load.eth_txd2_i = driver.eth_txd2_o; \
  assign load.eth_txd3_i = driver.eth_txd3_o; \

`define ASSIGN_PERIPHS_UART7_PAD2SOC(load, driver) \
  assign load.rx_i = driver.rx_o; \

`define ASSIGN_PERIPHS_UART7_SOC2PAD(load, driver) \
  assign load.tx_i = driver.tx_o; \

`define ASSIGN_PERIPHS_I2C5_PAD2SOC(load, driver) \
  assign load.scl_i = driver.scl_o; \
  assign load.sda_i = driver.sda_o; \

`define ASSIGN_PERIPHS_I2C5_SOC2PAD(load, driver) \
  assign load.scl_i = driver.scl_o; \
  assign load.scl_oe_i = driver.scl_oe_o; \
  assign load.sda_i = driver.sda_o; \
  assign load.sda_oe_i = driver.sda_oe_o; \


