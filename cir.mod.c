#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x115e2a2b, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x149c7752, __VMLINUX_SYMBOL_STR(param_ops_uint) },
	{ 0x60ee9172, __VMLINUX_SYMBOL_STR(param_ops_bool) },
	{ 0x55ae5ccb, __VMLINUX_SYMBOL_STR(seq_read) },
	{ 0xccbf1a65, __VMLINUX_SYMBOL_STR(seq_lseek) },
	{ 0x6cb5e52b, __VMLINUX_SYMBOL_STR(platform_driver_unregister) },
	{ 0x6f33b43b, __VMLINUX_SYMBOL_STR(__platform_driver_register) },
	{ 0x8e865d3c, __VMLINUX_SYMBOL_STR(arm_delay_ops) },
	{ 0xead9a25d, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x815588a6, __VMLINUX_SYMBOL_STR(clk_enable) },
	{ 0x50dac6ed, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0xc2165d85, __VMLINUX_SYMBOL_STR(__arm_iounmap) },
	{ 0xb2585f71, __VMLINUX_SYMBOL_STR(rc_register_device) },
	{ 0x7caa2a99, __VMLINUX_SYMBOL_STR(rc_allocate_device) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xadf42bd5, __VMLINUX_SYMBOL_STR(__request_region) },
	{ 0x658dac4b, __VMLINUX_SYMBOL_STR(proc_create_data) },
	{ 0x49a5920e, __VMLINUX_SYMBOL_STR(proc_mkdir) },
	{ 0x7c9a7371, __VMLINUX_SYMBOL_STR(clk_prepare) },
	{ 0x2a827361, __VMLINUX_SYMBOL_STR(__pm_runtime_resume) },
	{ 0x83cf8444, __VMLINUX_SYMBOL_STR(pm_runtime_enable) },
	{ 0xe2fe0167, __VMLINUX_SYMBOL_STR(pm_runtime_irq_safe) },
	{ 0x5ce0b889, __VMLINUX_SYMBOL_STR(pm_runtime_set_autosuspend_delay) },
	{ 0x6f796d72, __VMLINUX_SYMBOL_STR(__pm_runtime_use_autosuspend) },
	{ 0x42a28178, __VMLINUX_SYMBOL_STR(clk_get) },
	{ 0xffd0a56, __VMLINUX_SYMBOL_STR(of_match_device) },
	{ 0x1e83fc18, __VMLINUX_SYMBOL_STR(irq_of_parse_and_map) },
	{ 0x84c60e5b, __VMLINUX_SYMBOL_STR(of_property_read_u32_array) },
	{ 0x57971f61, __VMLINUX_SYMBOL_STR(of_alias_get_id) },
	{ 0x2b0843a2, __VMLINUX_SYMBOL_STR(devm_ioremap_nocache) },
	{ 0x5d62408b, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0xebce902e, __VMLINUX_SYMBOL_STR(platform_get_resource) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0xf612f1da, __VMLINUX_SYMBOL_STR(ir_raw_event_store) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xb2d48a2e, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0xe530800a, __VMLINUX_SYMBOL_STR(dev_pm_set_dedicated_wake_irq) },
	{ 0xe707d823, __VMLINUX_SYMBOL_STR(__aeabi_uidiv) },
	{ 0x69fdb86a, __VMLINUX_SYMBOL_STR(ir_raw_event_set_idle) },
	{ 0xebca8656, __VMLINUX_SYMBOL_STR(ir_raw_event_handle) },
	{ 0xc0eebfb9, __VMLINUX_SYMBOL_STR(ir_raw_event_store_with_filter) },
	{ 0x788fe103, __VMLINUX_SYMBOL_STR(iomem_resource) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0x28cd4fa0, __VMLINUX_SYMBOL_STR(remove_proc_entry) },
	{ 0x9bce482f, __VMLINUX_SYMBOL_STR(__release_region) },
	{ 0x2e8647f5, __VMLINUX_SYMBOL_STR(devm_iounmap) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0xb077e70a, __VMLINUX_SYMBOL_STR(clk_unprepare) },
	{ 0xb6e6d99d, __VMLINUX_SYMBOL_STR(clk_disable) },
	{ 0x139cede9, __VMLINUX_SYMBOL_STR(device_init_wakeup) },
	{ 0x4fc3ac68, __VMLINUX_SYMBOL_STR(rc_unregister_device) },
	{ 0x4205ad24, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x801678, __VMLINUX_SYMBOL_STR(flush_scheduled_work) },
	{ 0xf2d222f9, __VMLINUX_SYMBOL_STR(__pm_runtime_disable) },
	{ 0xa1cbf468, __VMLINUX_SYMBOL_STR(__pm_runtime_idle) },
	{ 0x43b0c9c3, __VMLINUX_SYMBOL_STR(preempt_schedule) },
	{ 0x822137e2, __VMLINUX_SYMBOL_STR(arm_heavy_mb) },
	{ 0x226e3e33, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x531787a7, __VMLINUX_SYMBOL_STR(seq_open) },
	{ 0xad6a46b0, __VMLINUX_SYMBOL_STR(kmem_cache_alloc) },
	{ 0xce8b3243, __VMLINUX_SYMBOL_STR(seq_printf) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Ccir-uart*");

MODULE_INFO(srcversion, "52A32DB7129A4C6FA0F0866");
