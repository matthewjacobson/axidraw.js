'use strict';

// ******************************************************************************** //
// ********************************** device.py *********************************** //
// ******************************************************************************** //
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
		this.inputDone;
		this.outputDone;
		this.inputStream;
		this.outputStream;
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
		this.error = [0, 0];
	}

	async connect() {
		let filter = [{ usbVendorId: VID, usbProductId: PID }];
		this.port = await navigator.serial.requestPort({filters: filter});
		await this.port.open({baudRate: 9600});
		const encoder = new TextEncoderStream();
		this.outputDone = encoder.readable.pipeTo(this.port.writable);
		this.outputStream = encoder.writable;
		let decoder = new TextDecoderStream();
		this.inputDone = this.port.readable.pipeTo(decoder.writable);
		this.inputStream = decoder.readable;
		this.reader = this.inputStream.getReader();
		this.readLoop();
	}

	async disconnect() {
		if (this.reader) {
			await this.reader.cancel();
			await this.inputDone.catch(() => {});
			this.reader = null;
			this.inputDone = null;
		}
		if (this.outputStream) {
			await this.outputStream.getWriter().close();
			await this.outputDone;
			this.outputStream = null;
			this.outputDone = null;
		}
		await this.port.close();
		this.port = null;
	}

	async readLoop() {
		while (true) {
			const { value, done } = await this.reader.read();
			if (value) {
				console.log('[READ]', value);
			}
			if (done) {
				console.log('[readLoop] DONE', done);
				this.reader.releaseLock();
				break;
			}
		}
	}

	write(...cmd) {
		const writer = this.outputStream.getWriter();
		let line = cmd.join(',');
		console.log('[SEND]', line);
		writer.write(line + '\n');
		writer.releaseLock();
		return;
	}

	// utilities
	makePlanner(jog = false) {
		let a = this.acceleration;
		let vmax = this.maxVelocity;
		let cf = this.cornerFactor;
		if (jog) {
			a = this.jogAcceleration;
			vmax = this.jogMaxVelocity;
		}
		return new Planner(a, vmax, cf);
	}

	// higher level functions
	reset() {
		this.disableMotors();
		this.penUp();
	}
	draw(paths) {
		this.enableMotors();
		this.runDrawing(paths);
		this.disableMotors();
	}

	// miscellaneous commands
	version() { return this.write('V'); }

	// motor functions
	enableMotors() { return this.write('EM', MICROSTEPPING_MODE, MICROSTEPPING_MODE); }
	disableMotors() { return this.write('EM', 0, 0); }
	motorStatus() { return this.write('QM'); }
	zeroPosition() { return this.write('CS'); }
	stepperMove(duration, a, b) { return this.write('XM', duration, a, b); }
	wait() { return this.write('XM', 10, 0, 0); }

	// drawing functions
	runPlan(plan) {
		let stepMs = TIMESLICE_MS;
		let stepS = stepMs / 1000;
		let t = 0;
		while (t < plan.t) {
			let i1 = plan.instant(t);
			let i2 = plan.instant(t + stepS);
			let d = i2.p.sub(i1.p);
			let mx = d.x * this.stepsPerUnit + this.error[0];
			let my = d.y * this.stepsPerUnit + this.error[1];
			let sx = Math.trunc(mx);
			let sy = Math.trunc(my);
			this.error = [mx - sx, my - sy];
			this.stepperMove(stepMs, sx, sy);
			t += stepS;
		}
	}
	runPath(path, jog = false) {
		let planner = this.makePlanner(jog);
		let plan = planner.plan(path);
		this.runPlan(plan);
	}
	runDrawing(paths) {
		this.penUp();
		let position = [0, 0];
		for (let path of paths) {
			this.runPath([position, path[0]], true);
			this.penDown();
			this.runPath(path);
			this.penUp();
			position = path[path.length - 1];
		}
		this.runPath([position, [0, 0]], true);
	}

	// pen functions
	penUp() {
		let delta = Math.abs(this.penUpPosition - this.penDownPosition);
		let duration = Math.floor(1000 * delta / this.penUpSpeed);
		let delay = Math.max(0, duration + this.penUpDelay);
		return this.write('SP,1,' + delay);
	}
	penDown() {
		let delta = Math.abs(this.penUpPosition - this.penDownPosition);
		let duration = Math.floor(1000 * delta / this.penDownSpeed);
		let delay = Math.max(0, duration + this.penDownDelay);
		return this.write('SP,0,' + delay);
	}

}



// ******************************************************************************** //
// ********************************** planner.py ********************************** //
// ******************************************************************************** //

// a planner computes a motion profile for a list of (x, y) points
class Planner {
	constructor(acceleration, maxVelocity, cornerFactor) {
		this.acceleration = acceleration;
		this.maxVelocity = maxVelocity;
		this.cornerFactor = cornerFactor;
	}
	plan(points) { return constantAccelerationPlan(points, this.acceleration, this.maxVelocity, this.cornerFactor); }
	planAll(paths) { return paths.map(path => this.plan(path)); }
}

// a plan is a motion profile generated by the planner
class Plan {
	constructor(blocks) {
		this.blocks = blocks;
		this.ts = [];
		this.ss = [];
		let t = 0;
		let s = 0;
		for (let b of blocks) {
			this.ts.push(t);
			this.ss.push(s);
			t += b.t;
			s += b.s;
		}
		this.t = t;
		this.s = s;
	}
	instant(t) {
		let tClamp = Math.max(0, Math.min(this.t, t));
		let i = bisect(this.ts, tClamp) - 1;
		return this.blocks[i].instant(t - this.ts[i], this.ts[i], this.ss[i]);
	}
}

// a block is a constant acceleration for a duration of time
class Block {
	constructor(a, t, vi, p1, p2) {
		this.a = a;
		this.t = t;
		this.vi = vi;
		this.p1 = p1;
		this.p2 = p2;
		this.s = p1.distance(p2);
	}
	instant(t, dt = 0, ds = 0) {
		let tClamp = Math.max(0, Math.min(this.t, t));
		let a = this.a;
		let v = this.vi + this.a * tClamp;
		let s = this.vi * tClamp + this.a * tClamp * tClamp / 2;
		s = Math.max(0, Math.min(this.s, s));
		let p = this.p1.lerps(this.p2, s);
		return new Instant(t + dt, p, s + ds, v, a);
	}
}

// an instant gives position, velocity, etc. at a single point in time
class Instant {
	constructor(t, p, s, v, a) {
		this.t = t;
		this.p = p;
		this.s = s;
		this.v = v;
		this.a = a;
	}
}

class Point {
	constructor(x, y) {
		this.x = x;
		this.y = y;
	}
	length() { return Math.hypot(this.x, this.y); }
	normalize() {
		let d = this.length();
		if (d == 0) return new Point(0, 0);
		return new Point(this.x / d, this.y / d);
	}
	distance(other) { return Math.hypot(this.x - other.x, this.y - other.y); }
	distanceSquared(other) { return Math.pow(this.x - other.x, 2) + Math.pow(this.y - other.y, 2); }
	add(other) { return new Point(this.x + other.x, this.y + other.y); }
	sub(other) { return new Point(this.x - other.x, this.y - other.y); }
	mul(factor) { return new Point(this.x * factor, this.y * factor); }
	dot(other) { return this.x * other.x + this.y * other.y; }
	lerps(other, s) {
		let v = other.sub(this).normalize();
		return this.add(v.mul(s));
	}
	segmentDistance(v, w) {
		let l2 =  v.distanceSquared(w);
		if (l2 == 0) return this.distance(v);
		let t = ((this.x - v.x) * (w.x - v.x) + (this.y - v.y) * (w.y - v.y)) / l2;
		t = Math.max(0, Math.min(1, t));
		let x = v.x + t * (w.x - v.x);
		let y = v.y + t * (w.y - v.y);
		let q = new Point(x, y);
		return this.distance(q);
	}
}

class Triangle {
	constructor(s1, s2, t1, t2, vmax, p1, p2, p3) {
		this.s1 = s1;
		this.s2 = s2;
		this.t1 = t1;
		this.t2 = t2;
		this.vmax = vmax;
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;
	}
}

function triangle(s, vi, vf, a, p1, p3) {
	let s1 = (2 * a * s + vf * vf - vi * vi) / (4 * a);
	let s2 = s - s1;
	let vmax = Math.sqrt(vi * vi + 2 * a * s1);
	let t1 = (vmax - vi) / a;
	let t2 = (vf - vmax) / -a;
	let p2 = p1.lerps(p3, s1);
	return new Triangle(s1, s2, t1, t2, vmax, p1, p2, p3);
}

class Trapezoid {
	constructor(s1, s2, s3, t1, t2, t3, p1, p2, p3, p4) {
		this.s1 = s1;
		this.s2 = s2;
		this.s3 = s3;
		this.t1 = t1;
		this.t2 = t2;
		this.t3 = t3;
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;
		this.p4 = p4;
	}
}

function trapezoid(s, vi, vmax, vf, a, p1, p4) {
	let t1 = (vmax - vi) / a;
	let s1 = (vmax + vi) / 2 * t1;
	let t3 = (vf - vmax) / -a;
	let s3 = (vf + vmax) / 2 * t3;
	let s2 = s - s1 - s3;
	let t2 = s2 / vmax;
	let p2 = p1.lerps(p4, s1);
	let p3 = p1.lerps(p4, s - s3);
	return new Trapezoid(s1, s2, s3, t1, t2, t3, p1, p2, p3, p4);
}

function cornerVelocity(s1, s2, vmax, a, delta) {
	// compute a maximum velocity at the corner of two segments
	// https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/
	let cosine = -s1.vector.dot(s2.vector);
	if (Math.abs(cosine - 1) < EPS) return 0;
	let sine = Math.sqrt((1 - cosine) / 2);
	if (Math.abs(sine - 1) < EPS) return vmax;
	let v = Math.sqrt((a * delta * sine) / (1 - sine));
	return Math.min(v, vmax);
}

// a segment is a line segment between two points, which will be broken up into blocks by the planner
class Segment {
	constructor(p1, p2) {
		this.p1 = p1;
		this.p2 = p2;
		this.length = p1.distance(p2);
		this.vector = p2.sub(p1).normalize();
		this.maxEntryVelocity = 0;
		this.entryVelocity = 0;
		this.blocks = [];
	}
}

class Throttler {
	constructor(points, vmax, dt, threshold) {
		this.points = points;
		this.vmax = vmax;
		this.dt = dt;
		this.threshold = threshold;
		this.distances = [];
		let prev = points[0];
		let d = 0;
		for (let point of points) {
			d += prev.distance(point);
			this.distances.push(d);
			prev = point;
		}
	}
	lookup(d) { return bisect(this.distances, d) - 1; }
	isFeasible(i0, v) {
		let d = v * this.dt;
		let x0 = this.distances[i0];
		let x1 = x0 + d;
		let i1 = this.lookup(x1);
		if (i0 == i1) return true;
		let p0 = this.points[i0];
		let p10 = this.points[i1];
		let p11 = p10;
		if (this.points.length > i1 + 1) p11 = this.points[i1 + 1];
		let s = x1 - this.distances[i1];
		let p1 = p10.lerps(p11, s);
		let i = i0 + 1;
		while (i <= i1) {
			let p = this.points[i];
			if (p.segmentDistance(p0, p1) > this.threshold) return false;
			i += 1;
		}
		return true;
	}
	computeMaxVelocity(index) {
		if (this.isFeasible(index, this.vmax)) return this.vmax;
		let lo = 0;
		let hi = this.vmax;
		for (let i = 0; i < 16; i++) {
			let v = (lo + hi) / 2;
			if (this.isFeasible(index, v)) lo = v;
			else hi = v;
		}
		return lo;
	}
	computeMaxVelocities() {
		let maxVelocities = [];
		for (let i = 0; i < this.points.length; i++) {
			maxVelocities.push(this.computeMaxVelocity(i));
		}
		return maxVelocities;
	}
}

function constantAccelerationPlan(points, a, vmax, cf) {

	// make sure points are Point objects
	points = points.map(p => new Point(p[0], p[1]));

	// the throttler reduces speeds based on the discrete timeslicing nature of the device
	let throttler = new Throttler(points, vmax, 0.02, 0.001);
	let maxVelocities = throttler.computeMaxVelocities();

	// create segments for each consecutive pair of points
	let segments = [];
	for (let i = 0; i < points.length - 1; i++) segments.push(new Segment(points[i], points[i + 1]));

	// compute a max_entry_velocity for each segment based on the angle formed by the two segments at the vertex
	for (let i = 0; i < segments.length - 1; i++) {
		let v = maxVelocities[i];
		let s1 = segments[i];
		let s2 = segments[i + 1];
		s1.maxEntryVelocity = Math.min(s1.maxEntryVelocity, v);
		s2.maxEntryVelocity = cornerVelocity(s1, s2, vmax, a, cf);
	}

	// add a dummy segment at the end to force a final velocity of zero
	segments.push(new Segment(points[points.length - 1], points[points.length - 1]));

	// loop over segments
	let i = 0;
	while (i < segments.length - 1) {
		// pull out some variables
		let segment = segments[i];
		let nextSegment = segments[i + 1];
		let s = segment.length;
		let vi = segment.entryVelocity;
		let vexit = nextSegment.maxEntryVelocity;
		let p1 = segment.p1;
		let p2 = segment.p2;

		// determine which profile to use for this segment
		let m = triangle(s, vi, vexit, a, p1, p2);
		if (m.s1 < -EPS) {
			// too fast! update max_entry_velocity and backtrack
			segment.maxEntryVelocity = Math.sqrt(vexit * vexit + 2 * a * s);
			i -= 1;
		} else if (m.s2 < 0) {
			// accelerate
			let vf = Math.sqrt(vi * vi + 2 * a * s);
			let t = (vf - vi) / a;
			segment.blocks = [new Block(a, t, vi, p1, p2)];
			nextSegment.entryVelocity = vf;
			i += 1
		} else if (m.vmax > vmax) {
			// accelerate, cruise, decelerate
			let z = trapezoid(s, vi, vmax, vexit, a, p1, p2);
			segments.blocks = [
				new Block(a, z.t1, vi, z.p1, z.p2),
				new Block(0, z.t2, vmax, z.p2, z.p3),
				new Block(-a, z.t3, vmax, z.p3, z.p4)
			];
			nextSegment.entryVelocity = vexit;
			i += 1;
		} else {
			// accelerate, decelerate
			segment.blocks = [
				new Block(a, m.t1, vi, m.p1, m.p2),
				new Block(-a, m.t2, m.vmax, m.p2, m.p3)
			];
			nextSegment.entryVelocity = vexit;
			i += 1;
		}
	}

	// concatenate all of the blocks
	let blocks = [];
	for (let segment of segments) {
		for (let block of segment.blocks) {
			if (block.t > EPS) {
				// filter out zero-duration blocks
				blocks.push(block);
			}
		}
	}

	return new Plan(blocks);
}

function bisect(a, v, lo = 0, hi = null) {
	if (lo < 0) return null;
	if (!hi) hi = a.length;
	while (lo < hi) {
		let mid = Math.floor((lo + hi) / 2);
		if (v < a[mid]) hi = mid;
		else lo = mid  + 1;
	}
	return lo;
}
