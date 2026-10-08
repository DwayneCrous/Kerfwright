/// Device + queue + adapter bundle.
pub struct Gpu {
    pub instance: wgpu::Instance,
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

impl Gpu {
    pub fn describe(&self) -> String {
        let i = self.adapter.get_info();
        format!(
            "{} ({:?}, {:?}, driver: {} {})",
            i.name, i.backend, i.device_type, i.driver, i.driver_info
        )
    }

    /// Create a device. `surface_target` is set by the windowed front end so the
    /// adapter can present to the window.
    pub fn new(instance: wgpu::Instance, surface: Option<&wgpu::Surface<'_>>) -> Gpu {
        let adapter = pollster::block_on(wgpu::util::initialize_adapter_from_env_or_default(
            &instance, surface,
        ))
        .expect("no suitable GPU adapter found");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("kerfwright"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::default().using_resolution(adapter.limits()),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .expect("failed to create device");
        Gpu {
            instance,
            adapter,
            device,
            queue,
        }
    }

    pub fn headless() -> Gpu {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        Gpu::new(instance, None)
    }
}
