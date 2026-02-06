use std::env;

/// All configuration for the transport server, loaded from environment variables.
pub struct Config {
    pub shm_path: String,
    pub shm_size: usize,
    pub nats_url: String,
    pub listen_addr: String,
    pub broadcast_capacity: usize,
    pub crc_enabled: bool,
    pub idr_coalesce_ms: u64,
    pub idr_timeout_ms: u64,
    pub telemetry_subjects: String,
    pub telemetry_max_size: usize,
}

impl Config {
    pub fn from_env() -> Self {
        Self {
            shm_path: env::var("SDR_SHM_PATH")
                .unwrap_or_else(|_| "/dev/shm/sdr_os_ipc/frames".into()),
            shm_size: env::var("SDR_SHM_SIZE")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(4 * 1024 * 1024),
            nats_url: env::var("SDR_NATS_URL")
                .unwrap_or_else(|_| "nats://nats:4222".into()),
            listen_addr: env::var("SDR_LISTEN_ADDR")
                .unwrap_or_else(|_| "0.0.0.0:8080".into()),
            broadcast_capacity: env::var("SDR_BROADCAST_CAPACITY")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(64),
            crc_enabled: env::var("SDR_CRC_ENABLED")
                .map(|v| v != "false" && v != "0")
                .unwrap_or(true),
            idr_coalesce_ms: env::var("SDR_IDR_COALESCE_MS")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(500),
            idr_timeout_ms: env::var("SDR_IDR_TIMEOUT_MS")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(5000),
            telemetry_subjects: env::var("SDR_TELEMETRY_SUBJECTS")
                .unwrap_or_else(|_| "telemetry.>".into()),
            telemetry_max_size: env::var("SDR_TELEMETRY_MAX_SIZE")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(65536),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // NOTE: env::set_var/remove_var are process-global, so these tests must
    // not run in parallel.  We combine them into a single test function.
    #[test]
    fn test_config_from_env() {
        // --- defaults ---
        for key in &[
            "SDR_SHM_PATH", "SDR_SHM_SIZE", "SDR_NATS_URL", "SDR_LISTEN_ADDR",
            "SDR_BROADCAST_CAPACITY", "SDR_CRC_ENABLED", "SDR_IDR_COALESCE_MS",
            "SDR_IDR_TIMEOUT_MS", "SDR_TELEMETRY_SUBJECTS", "SDR_TELEMETRY_MAX_SIZE",
        ] {
            env::remove_var(key);
        }

        let cfg = Config::from_env();
        assert_eq!(cfg.shm_path, "/dev/shm/sdr_os_ipc/frames");
        assert_eq!(cfg.shm_size, 4 * 1024 * 1024);
        assert_eq!(cfg.nats_url, "nats://nats:4222");
        assert_eq!(cfg.listen_addr, "0.0.0.0:8080");
        assert_eq!(cfg.broadcast_capacity, 64);
        assert!(cfg.crc_enabled);
        assert_eq!(cfg.idr_coalesce_ms, 500);
        assert_eq!(cfg.idr_timeout_ms, 5000);
        assert_eq!(cfg.telemetry_subjects, "telemetry.>");
        assert_eq!(cfg.telemetry_max_size, 65536);

        // --- CRC disabled ---
        env::set_var("SDR_CRC_ENABLED", "false");
        let cfg = Config::from_env();
        assert!(!cfg.crc_enabled);
        env::remove_var("SDR_CRC_ENABLED");
    }
}
