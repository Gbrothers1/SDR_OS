// Sound effect synthesizer for UI interactions
class SoundEffects {
    constructor() {
        this.audioContext = null;
        this.masterGain = null;
        this.initialized = false;
    }

    async initializeAudio() {
        if (this.initialized) return;
        
        try {
            this.audioContext = new (window.AudioContext || window.webkitAudioContext)();
            this.masterGain = this.audioContext.createGain();
            this.masterGain.gain.value = 0.15; // Master volume
            this.masterGain.connect(this.audioContext.destination);
            this.initialized = true;
        } catch (error) {
            console.warn('Audio initialization failed:', error);
        }
    }

    async ensureAudioContext() {
        if (!this.initialized) {
            await this.initializeAudio();
        }
        
        if (this.audioContext?.state === 'suspended') {
            await this.audioContext.resume();
        }
        
        return this.initialized;
    }

    // Button click sound with sci-fi feel
    async playButtonClick() {
        if (!await this.ensureAudioContext()) return;
        
        const now = this.audioContext.currentTime;
        
        // Create oscillators for a complex sound
        const osc1 = this.audioContext.createOscillator();
        const osc2 = this.audioContext.createOscillator();
        
        // Create gain nodes for envelope
        const gainNode1 = this.audioContext.createGain();
        const gainNode2 = this.audioContext.createGain();
        
        // Configure oscillators
        osc1.type = 'sine';
        osc1.frequency.setValueAtTime(2000, now);
        osc1.frequency.exponentialRampToValueAtTime(1200, now + 0.1);
        
        osc2.type = 'square';
        osc2.frequency.setValueAtTime(1800, now);
        osc2.frequency.exponentialRampToValueAtTime(600, now + 0.1);
        
        // Configure gain (volume envelope)
        gainNode1.gain.setValueAtTime(0, now);
        gainNode1.gain.linearRampToValueAtTime(0.3, now + 0.02);
        gainNode1.gain.exponentialRampToValueAtTime(0.001, now + 0.1);
        
        gainNode2.gain.setValueAtTime(0, now);
        gainNode2.gain.linearRampToValueAtTime(0.2, now + 0.02);
        gainNode2.gain.exponentialRampToValueAtTime(0.001, now + 0.08);
        
        // Connect nodes
        osc1.connect(gainNode1);
        osc2.connect(gainNode2);
        gainNode1.connect(this.masterGain);
        gainNode2.connect(this.masterGain);
        
        // Start and stop
        osc1.start(now);
        osc2.start(now);
        osc1.stop(now + 0.1);
        osc2.stop(now + 0.1);
    }

    // D-pad button sound
    async playDpadClick() {
        if (!await this.ensureAudioContext()) return;
        
        const now = this.audioContext.currentTime;
        
        const osc = this.audioContext.createOscillator();
        const gainNode = this.audioContext.createGain();
        
        osc.type = 'sine';
        osc.frequency.setValueAtTime(1500, now);
        osc.frequency.exponentialRampToValueAtTime(800, now + 0.05);
        
        gainNode.gain.setValueAtTime(0, now);
        gainNode.gain.linearRampToValueAtTime(0.2, now + 0.02);
        gainNode.gain.exponentialRampToValueAtTime(0.001, now + 0.05);
        
        osc.connect(gainNode);
        gainNode.connect(this.masterGain);
        
        osc.start(now);
        osc.stop(now + 0.05);
    }

    // Face button sound
    async playFaceButtonClick() {
        if (!await this.ensureAudioContext()) return;
        
        const now = this.audioContext.currentTime;
        
        const osc1 = this.audioContext.createOscillator();
        const osc2 = this.audioContext.createOscillator();
        const gainNode = this.audioContext.createGain();
        
        osc1.type = 'sine';
        osc1.frequency.setValueAtTime(2200, now);
        osc1.frequency.exponentialRampToValueAtTime(1800, now + 0.08);
        
        osc2.type = 'triangle';
        osc2.frequency.setValueAtTime(2400, now);
        osc2.frequency.exponentialRampToValueAtTime(2000, now + 0.08);
        
        gainNode.gain.setValueAtTime(0, now);
        gainNode.gain.linearRampToValueAtTime(0.15, now + 0.02);
        gainNode.gain.exponentialRampToValueAtTime(0.001, now + 0.08);
        
        osc1.connect(gainNode);
        osc2.connect(gainNode);
        gainNode.connect(this.masterGain);
        
        osc1.start(now);
        osc2.start(now);
        osc1.stop(now + 0.08);
        osc2.stop(now + 0.08);
    }

    // Menu button sound
    async playMenuButtonClick() {
        if (!await this.ensureAudioContext()) return;
        
        const now = this.audioContext.currentTime;
        
        const osc1 = this.audioContext.createOscillator();
        const osc2 = this.audioContext.createOscillator();
        const gainNode = this.audioContext.createGain();
        
        osc1.type = 'sine';
        osc1.frequency.setValueAtTime(1600, now);
        osc1.frequency.exponentialRampToValueAtTime(1200, now + 0.15);
        
        osc2.type = 'square';
        osc2.frequency.setValueAtTime(800, now);
        osc2.frequency.exponentialRampToValueAtTime(400, now + 0.15);
        
        gainNode.gain.setValueAtTime(0, now);
        gainNode.gain.linearRampToValueAtTime(0.2, now + 0.05);
        gainNode.gain.exponentialRampToValueAtTime(0.001, now + 0.15);
        
        osc1.connect(gainNode);
        osc2.connect(gainNode);
        gainNode.connect(this.masterGain);
        
        osc1.start(now);
        osc2.start(now);
        osc1.stop(now + 0.15);
        osc2.stop(now + 0.15);
    }
}

// Create a singleton instance
const soundEffects = new SoundEffects();
export default soundEffects; 