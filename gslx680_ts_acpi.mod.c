#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

#ifdef CONFIG_UNWINDER_ORC
#include <asm/orc_header.h>
ORC_HEADER;
#endif

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_MITIGATION_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif



static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xf0fdf6cb, "__stack_chk_fail" },
	{ 0xfcec0987, "enable_irq" },
	{ 0xce2840e7, "irq_set_irq_wake" },
	{ 0x3ce4ca6f, "disable_irq" },
	{ 0x3901b934, "input_event" },
	{ 0x6be40a88, "input_mt_report_slot_state" },
	{ 0xa72a3b61, "input_mt_sync_frame" },
	{ 0xd8ccd803, "input_mt_assign_slots" },
	{ 0x49e472e, "i2c_del_driver" },
	{ 0x65487097, "__x86_indirect_thunk_rax" },
	{ 0xe3e46924, "devm_kmalloc" },
	{ 0x9166fada, "strncpy" },
	{ 0x224537cc, "request_firmware" },
	{ 0xa6ec47c1, "devm_input_allocate_device" },
	{ 0xf60cabf7, "input_set_capability" },
	{ 0xb7aa795f, "input_set_abs_params" },
	{ 0xf7745958, "input_mt_init_slots" },
	{ 0x7f455f82, "input_register_device" },
	{ 0xc6d09aa9, "release_firmware" },
	{ 0xc3651746, "devm_gpiod_get_index" },
	{ 0xcccf16ad, "devm_request_threaded_irq" },
	{ 0xb1185642, "device_set_wakeup_capable" },
	{ 0x1ccc2c29, "device_wakeup_enable" },
	{ 0x760cfbdf, "_dev_info" },
	{ 0x65e6bfa7, "param_ops_charp" },
	{ 0xbdfb6dbb, "__fentry__" },
	{ 0x386de390, "i2c_register_driver" },
	{ 0x5b8239ca, "__x86_return_thunk" },
	{ 0x32bef6d4, "gpiod_set_value_cansleep" },
	{ 0xc3055d20, "usleep_range_state" },
	{ 0x5190f1, "is_acpi_device_node" },
	{ 0xcb733bf2, "acpi_bus_set_power" },
	{ 0xff7dc8fa, "_dev_warn" },
	{ 0x73686b7b, "__dynamic_dev_dbg" },
	{ 0x83f35571, "_dev_err" },
	{ 0xaed5e907, "i2c_transfer_buffer_flags" },
	{ 0x725179e9, "module_layout" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("acpi*:MSSL1680:*");
MODULE_ALIAS("acpi*:MSSL3680:*");
MODULE_ALIAS("acpi*:PNP1680:*");
MODULE_ALIAS("acpi*:PNP3680:*");
MODULE_ALIAS("i2c:gslx680");

MODULE_INFO(srcversion, "95C890A7DBC947ABCB2EE79");
