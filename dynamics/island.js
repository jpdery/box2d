// requires 
var b2ContactImpulse = require('./contact-impulse.js'); 
var b2Settings = require('../common/settings.js'); 
var b2Math = require('../common/math/math.js'); 
var b2Island = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Island.prototype.__constructor = function () {
		this.m_bodies = new Array();
		this.m_contacts = new Array();
		this.m_joints = new Array();
	}
b2Island.prototype.__varz = function(){
}
// static methods
// static attributes
b2Island.s_impulse =  new b2ContactImpulse();
// methods
b2Island.prototype.Initialize = function (
	bodyCapacity,
	contactCapacity,
	jointCapacity,
	allocator,
	listener,
	contactSolver) {
		var i = 0;
		
		this.m_bodyCapacity = bodyCapacity;
		this.m_contactCapacity = contactCapacity;
		this.m_jointCapacity	 = jointCapacity;
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
		
		this.m_allocator = allocator;
		this.m_listener = listener;
		this.m_contactSolver = contactSolver;
		
		for (i = this.m_bodies.length; i < bodyCapacity; i++)
			this.m_bodies[i] = null;
		
		for (i = this.m_contacts.length; i < contactCapacity; i++)
			this.m_contacts[i] = null;
		
		for (i = this.m_joints.length; i < jointCapacity; i++)
			this.m_joints[i] = null;
		
	}
b2Island.prototype.Clear = function () {
		this.m_bodyCount = 0;
		this.m_contactCount = 0;
		this.m_jointCount = 0;
	}
b2Island.prototype.Solve = function (step, gravity, allowSleep) {
		var i = 0;
		var j = 0;
		var b;
		var joint;
		
		
		for (i = 0; i < this.m_bodyCount; ++i)
		{
			b = this.m_bodies[i];
			
			if (b.GetType() != b2Body.b2_dynamicBody)
				continue;
			
			
			
			b.m_linearVelocity.x += step.dt * (gravity.x + b.m_invMass * b.m_force.x);
			b.m_linearVelocity.y += step.dt * (gravity.y + b.m_invMass * b.m_force.y);
			b.m_angularVelocity += step.dt * b.m_invI * b.m_torque;
			
			
			
			
			
			
			
			
			b.m_linearVelocity.Multiply( b2Math.Clamp(1.0 - step.dt * b.m_linearDamping, 0.0, 1.0) );
			b.m_angularVelocity *= b2Math.Clamp(1.0 - step.dt * b.m_angularDamping, 0.0, 1.0);
		}
		
		this.m_contactSolver.Initialize(step, this.m_contacts, this.m_contactCount, this.m_allocator);
		var contactSolver = this.m_contactSolver;

		
		contactSolver.InitVelocityConstraints(step);
		
		for (i = 0; i < this.m_jointCount; ++i)
		{
			joint = this.m_joints[i];
			joint.InitVelocityConstraints(step);
		}
		
		
		for (i = 0; i < step.velocityIterations; ++i)
		{	
			for (j = 0; j < this.m_jointCount; ++j)
			{
				joint = this.m_joints[j];
				joint.SolveVelocityConstraints(step);
			}
			
			contactSolver.SolveVelocityConstraints();
		}
		
		
		for (i = 0; i < this.m_jointCount; ++i)
		{
			joint = this.m_joints[i];
			joint.FinalizeVelocityConstraints();
		}
		contactSolver.FinalizeVelocityConstraints();
		
		
		for (i = 0; i < this.m_bodyCount; ++i)
		{
			b = this.m_bodies[i];
			
			if (b.GetType() == b2Body.b2_staticBody)
				continue;
				
			
			
			var translationX = step.dt * b.m_linearVelocity.x;
			var translationY = step.dt * b.m_linearVelocity.y;
			
			if ((translationX*translationX+translationY*translationY) > b2Settings.b2_maxTranslationSquared)
			{
				b.m_linearVelocity.Normalize();
				b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * step.inv_dt;
				b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * step.inv_dt;
			}
			var rotation = step.dt * b.m_angularVelocity;
			if (rotation * rotation > b2Settings.b2_maxRotationSquared)
			{
				if (b.m_angularVelocity < 0.0)
				{
					b.m_angularVelocity = -b2Settings.b2_maxRotation * step.inv_dt;
				}
				else
				{
					b.m_angularVelocity = b2Settings.b2_maxRotation * step.inv_dt;
				}
			}
			
			
			b.m_sweep.c0.SetV(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;
			
			
			
			b.m_sweep.c.x += step.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += step.dt * b.m_linearVelocity.y;
			b.m_sweep.a += step.dt * b.m_angularVelocity;
			
			
			b.SynchronizeTransform();
			
			
		}
		
		
		for (i = 0; i < step.positionIterations; ++i)
		{
			var contactsOkay = contactSolver.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
			
			var jointsOkay = true;
			for (j = 0; j < this.m_jointCount; ++j)
			{
				joint = this.m_joints[j];
				var jointOkay = joint.SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
				jointsOkay = jointsOkay && jointOkay;
			}
			
			if (contactsOkay && jointsOkay)
			{
				break;
			}
		}
		
		this.Report(contactSolver.m_constraints);
		
		if (allowSleep){
			
			var minSleepTime = Number.MAX_VALUE;
			
			var linTolSqr = b2Settings.b2_linearSleepTolerance * b2Settings.b2_linearSleepTolerance;
			var angTolSqr = b2Settings.b2_angularSleepTolerance * b2Settings.b2_angularSleepTolerance;
			
			for (i = 0; i < this.m_bodyCount; ++i)
			{
				b = this.m_bodies[i];
				if (b.GetType() == b2Body.b2_staticBody)
				{
					continue;
				}
				
				if ((b.m_flags & b2Body.e_allowSleepFlag) == 0)
				{
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				}
				
				if ((b.m_flags & b2Body.e_allowSleepFlag) == 0 ||
					b.m_angularVelocity * b.m_angularVelocity > angTolSqr ||
					b2Math.Dot(b.m_linearVelocity, b.m_linearVelocity) > linTolSqr)
				{
					b.m_sleepTime = 0.0;
					minSleepTime = 0.0;
				}
				else
				{
					b.m_sleepTime += step.dt;
					minSleepTime = b2Math.Min(minSleepTime, b.m_sleepTime);
				}
			}
			
			if (minSleepTime >= b2Settings.b2_timeToSleep)
			{
				for (i = 0; i < this.m_bodyCount; ++i)
				{
					b = this.m_bodies[i]; 
					b.SetAwake(false);
				}
			}
		}
	}
b2Island.prototype.SolveTOI = function (subStep) {
		var i = 0;
		var j = 0;
		this.m_contactSolver.Initialize(subStep, this.m_contacts, this.m_contactCount, this.m_allocator);
		var contactSolver = this.m_contactSolver;
		
		
		

		
		
		for (i = 0; i < this.m_jointCount;++i)
		{
			this.m_joints[i].InitVelocityConstraints(subStep);
		}
		
		
		
		for (i = 0; i < subStep.velocityIterations; ++i)
		{
			contactSolver.SolveVelocityConstraints();
			for (j = 0; j < this.m_jointCount;++j)
			{
				this.m_joints[j].SolveVelocityConstraints(subStep);
			}
		}
		
		
		
		
		
		for (i = 0; i < this.m_bodyCount; ++i)
		{
			var b = this.m_bodies[i];
			
			if (b.GetType() == b2Body.b2_staticBody)
				continue;
				
			
			
			var translationX = subStep.dt * b.m_linearVelocity.x;
			var translationY = subStep.dt * b.m_linearVelocity.y;
			
			if ((translationX*translationX+translationY*translationY) > b2Settings.b2_maxTranslationSquared)
			{
				b.m_linearVelocity.Normalize();
				b.m_linearVelocity.x *= b2Settings.b2_maxTranslation * subStep.inv_dt;
				b.m_linearVelocity.y *= b2Settings.b2_maxTranslation * subStep.inv_dt;
			}
			
			var rotation = subStep.dt * b.m_angularVelocity;
			if (rotation * rotation > b2Settings.b2_maxRotationSquared)
			{
				if (b.m_angularVelocity < 0.0)
				{
					b.m_angularVelocity = -b2Settings.b2_maxRotation * subStep.inv_dt;
				}
				else
				{
					b.m_angularVelocity = b2Settings.b2_maxRotation * subStep.inv_dt;
				}
			}
			
			
			b.m_sweep.c0.SetV(b.m_sweep.c);
			b.m_sweep.a0 = b.m_sweep.a;
			
			
			b.m_sweep.c.x += subStep.dt * b.m_linearVelocity.x;
			b.m_sweep.c.y += subStep.dt * b.m_linearVelocity.y;
			b.m_sweep.a += subStep.dt * b.m_angularVelocity;
			
			
			b.SynchronizeTransform();
			
			
		}
		
		
		var k_toiBaumgarte = 0.75;
		for (i = 0; i < subStep.positionIterations; ++i)
		{
			var contactsOkay = contactSolver.SolvePositionConstraints(k_toiBaumgarte);
			var jointsOkay = true;
			for (j = 0; j < this.m_jointCount;++j)
			{
				var jointOkay = this.m_joints[j].SolvePositionConstraints(b2Settings.b2_contactBaumgarte);
				jointsOkay = jointsOkay && jointOkay;
			}
			
			if (contactsOkay && jointsOkay)
			{
				break;
			}
		}
		this.Report(contactSolver.m_constraints);
	}
b2Island.prototype.Report = function (constraints) {
		if (this.m_listener == null)
		{
			return;
		}
		
		for (var i = 0; i < this.m_contactCount; ++i)
		{
			var c = this.m_contacts[i];
			var cc = constraints[ i ];
			
			for (var j = 0; j < cc.pointCount; ++j)
			{
				b2Island.s_impulse.normalImpulses[j] = cc.points[j].normalImpulse;
				b2Island.s_impulse.tangentImpulses[j] = cc.points[j].tangentImpulse;
			}
			this.m_listener.PostSolve(c, b2Island.s_impulse);
		}
	}
b2Island.prototype.AddBody = function (body) {
		
		body.m_islandIndex = this.m_bodyCount;
		this.m_bodies[this.m_bodyCount++] = body;
	}
b2Island.prototype.AddContact = function (contact) {
		
		this.m_contacts[this.m_contactCount++] = contact;
	}
b2Island.prototype.AddJoint = function (joint) {
		
		this.m_joints[this.m_jointCount++] = joint;
	}
// attributes
b2Island.prototype.m_allocator =  null;
b2Island.prototype.m_listener =  null;
b2Island.prototype.m_contactSolver =  null;
b2Island.prototype.m_bodies =  null;
b2Island.prototype.m_contacts =  null;
b2Island.prototype.m_joints =  null;
b2Island.prototype.m_bodyCount =  0;
b2Island.prototype.m_jointCount =  0;
b2Island.prototype.m_contactCount =  0;
b2Island.prototype.m_bodyCapacity =  0;
b2Island.prototype.m_contactCapacity =  0;
b2Island.prototype.m_jointCapacity =  0;// aliases  
b2Island.prototype.initialize = b2Island.prototype.Initialize; 
b2Island.prototype.clear = b2Island.prototype.Clear; 
b2Island.prototype.solve = b2Island.prototype.Solve; 
b2Island.prototype.solveTOI = b2Island.prototype.SolveTOI; 
b2Island.prototype.report = b2Island.prototype.Report; 
b2Island.prototype.addBody = b2Island.prototype.AddBody; 
b2Island.prototype.addContact = b2Island.prototype.AddContact; 
b2Island.prototype.addJoint = b2Island.prototype.AddJoint; 
// exports  
module.exports =  b2Island;
