let TIMESLICE_MS = 10;
let MICROSTEPPING_MODE = 1;
let STEP_DIVIDER = 2 ** (MICROSTEPPING_MODE - 1);
let STEPS_PER_INCH = 2032 / STEP_DIVIDER;
let STEPS_PER_MM = 80 / STEP_DIVIDER;
let PEN_UP_POSITION = 60;
let PEN_UP_SPEED = 150;
let PEN_UP_DELAY = 0;
let PEN_DOWN_POSITION = 40;
let PEN_DOWN_SPEED = 150;
let PEN_DOWN_DELAY = 0;
let ACCELERATION = 16;
let MAX_VELOCITY = 4;
let CORNER_FACTOR = 0.001;
let JOG_ACCELERATION = 16;
let JOG_MAX_VELOCITY = 8;

let EPS = 0.000000001;

let VID = 0x04d8;
let PID = 0xfd92;

class AxiDraw {

	constructor() {
		// web serial api stuff
		this.port;
		this.reader;
		this.writer;
		// axidraw config stuff
		this.stepsPerUnit = STEPS_PER_INCH;
		this.penUpPosition = PEN_UP_POSITION;
		this.penUpSpeed = PEN_UP_SPEED;
		this.penUpDelay = PEN_UP_DELAY;
		this.penDownPosition = PEN_DOWN_POSITION;
		this.penDownSpeed = PEN_DOWN_SPEED;
		this.penDownDelay = PEN_DOWN_DELAY;
		this.acceleration = ACCELERATION;
		this.maxVelocity = MAX_VELOCITY;
		this.cornerFactor = CORNER_FACTOR;
		this.jogAcceleration = JOG_ACCELERATION;
		this.jogMaxVelocity = JOG_MAX_VELOCITY;
	}

	async connect() {
		navigator.serial.requestPort({filters: [{ usbVendorId: VID, usbProductId: PID }]}).then(async (port) => {
			this.port = port;
			await this.port.open({baudRate: 9600});
			this.writer = this.port.writable.pipeThrough(new TextEncoderStream()).getWriter();
			this.reader = this.port.readable.pipeThrough(new TextDecoderStream()).getReader();
		})
	}

	async disconnect() {
		if (this.reader) {
			await this.reader.cancel();
			this.reader = null;
		}
		if (this.writer) {
			await this.writer.close();
			this.writer = null;
		}
		// close the port
		await this.port.close();
		this.port = null;
	}

	write(line) {
		this.writer.write(line);
		return this.read();
	}

	read() {
		this.reader.read().then(
			({value}) => { return value; },
			error => {
				console.error('error from read', error);
				return null;
			}
		);
	}
  
}
