//! Device creation for the render crate: asks for timestamp queries and every limit the adapter has
//! (the path tracer's node buffers exceed the default storage-buffer binding size).

pub struct RenderGpu {
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
    pub timestamps: bool,
}

impl RenderGpu {
    /// `None` when the machine has no usable adapter (tests then skip).
    pub fn try_new() -> Option<RenderGpu> {
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle());
        let adapter = pollster::block_on(wgpu::util::initialize_adapter_from_env_or_default(
            &instance, None,
        ))
        .ok()?;
        let timestamps = adapter.features().contains(wgpu::Features::TIMESTAMP_QUERY);
        let features = if timestamps {
            wgpu::Features::TIMESTAMP_QUERY
        } else {
            wgpu::Features::empty()
        };
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("kw-render"),
            required_features: features,
            required_limits: adapter.limits(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::Performance,
            trace: wgpu::Trace::Off,
        }))
        .ok()?;
        Some(RenderGpu {
            adapter,
            device,
            queue,
            timestamps,
        })
    }

    pub fn describe(&self) -> String {
        let i = self.adapter.get_info();
        format!(
            "{} ({:?}, {:?}, driver: {} {})",
            i.name, i.backend, i.device_type, i.driver, i.driver_info
        )
    }

    pub fn wait(&self) {
        self.device
            .poll(wgpu::PollType::wait_indefinitely())
            .expect("poll");
    }
}
