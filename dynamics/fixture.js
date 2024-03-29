// requires 
var b2AABB = require('../collision/aabb.js'); 
var b2FilterData = require('./filter-data.js'); 
var b2MassData = require('../collision/shapes/mass-data.js'); 
var b2Math = require('../common/math/math.js'); 
var b2Fixture = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2Fixture.prototype.__constructor = function () {
		this.m_aabb = new b2AABB();
		this.m_userData = null;
		this.m_body = null;
		this.m_next = null;
		
		this.m_shape = null;
		this.m_density = 0.0;
		
		this.m_friction = 0.0;
		this.m_restitution = 0.0;
	}
b2Fixture.prototype.__varz = function(){
this.m_filter =  new b2FilterData();
}
// static methods
// static attributes
// methods
b2Fixture.prototype.Create = function ( body, xf, def) {
		this.m_userData = def.userData;
		this.m_friction = def.friction;
		this.m_restitution = def.restitution;
		
		this.m_body = body;
		this.m_next = null;
		
		this.m_filter = def.filter.Copy();
		
		this.m_isSensor = def.isSensor;
		
		this.m_shape = def.shape.Copy();
		
		this.m_density = def.density;
	}
b2Fixture.prototype.Destroy = function () {
		
		
		
		
		this.m_shape = null;
	}
b2Fixture.prototype.CreateProxy = function (broadPhase, xf) {
		
		
		
		this.m_shape.ComputeAABB(this.m_aabb, xf);
		this.m_proxy = broadPhase.CreateProxy(this.m_aabb, this);
	}
b2Fixture.prototype.DestroyProxy = function (broadPhase) {
		if (this.m_proxy == null)
		{
			return;
		}
		
		
		broadPhase.DestroyProxy(this.m_proxy);
		this.m_proxy = null;
	}
b2Fixture.prototype.Synchronize = function (broadPhase, transform1, transform2) {
		if (!this.m_proxy)
			return;
			
		
		var aabb1 = new b2AABB();
		var aabb2 = new b2AABB();
		this.m_shape.ComputeAABB(aabb1, transform1);
		this.m_shape.ComputeAABB(aabb2, transform2);
		
		this.m_aabb.Combine(aabb1, aabb2);
		var displacement = b2Math.SubtractVV(transform2.position, transform1.position);
		broadPhase.MoveProxy(this.m_proxy, this.m_aabb, displacement);
	}
b2Fixture.prototype.GetType = function () {
		return this.m_shape.GetType();
	}
b2Fixture.prototype.GetShape = function () {
		return this.m_shape;
	}
b2Fixture.prototype.SetSensor = function (sensor) {
		if ( this.m_isSensor == sensor)
			return;
			
		this.m_isSensor = sensor;
		
		if (this.m_body == null)
			return;
			
		var edge = this.m_body.GetContactList();
		while (edge)
		{
			var contact = edge.contact;
			var fixtureA = contact.GetFixtureA();
			var fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.SetSensor(fixtureA.IsSensor() || fixtureB.IsSensor());
			edge = edge.next;
		}
		
	}
b2Fixture.prototype.IsSensor = function () {
		return this.m_isSensor;
	}
b2Fixture.prototype.SetFilterData = function (filter) {
		this.m_filter = filter.Copy();
		
		if (this.m_body)
			return;
			
		var edge = this.m_body.GetContactList();
		while (edge)
		{
			var contact = edge.contact;
			var fixtureA = contact.GetFixtureA();
			var fixtureB = contact.GetFixtureB();
			if (fixtureA == this || fixtureB == this)
				contact.FlagForFiltering();
			edge = edge.next;
		}
	}
b2Fixture.prototype.GetFilterData = function () {
		return this.m_filter.Copy();
	}
b2Fixture.prototype.GetBody = function () {
		return this.m_body;
	}
b2Fixture.prototype.GetNext = function () {
		return this.m_next;
	}
b2Fixture.prototype.GetUserData = function () {
		return this.m_userData;
	}
b2Fixture.prototype.SetUserData = function (data) {
		this.m_userData = data;
	}
b2Fixture.prototype.TestPoint = function (p) {
		return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
	}
b2Fixture.prototype.RayCast = function (output, input) {
		return this.m_shape.RayCast(output, input, this.m_body.GetTransform());
	}
b2Fixture.prototype.GetMassData = function (massData ) {
		if ( massData == null )
		{
			massData = new b2MassData();
		}
		this.m_shape.ComputeMass(massData, this.m_density);
		return massData;
	}
b2Fixture.prototype.SetDensity = function (density) {
		
		this.m_density = density;
	}
b2Fixture.prototype.GetDensity = function () {
		return this.m_density;
	}
b2Fixture.prototype.GetFriction = function () {
		return this.m_friction;
	}
b2Fixture.prototype.SetFriction = function (friction) {
		this.m_friction = friction;
	}
b2Fixture.prototype.GetRestitution = function () {
		return this.m_restitution;
	}
b2Fixture.prototype.SetRestitution = function (restitution) {
		this.m_restitution = restitution;
	}
b2Fixture.prototype.GetAABB = function () {
		return this.m_aabb;
	}
// attributes
b2Fixture.prototype.m_massData =  null;
b2Fixture.prototype.m_aabb =  null;
b2Fixture.prototype.m_density =  null;
b2Fixture.prototype.m_next =  null;
b2Fixture.prototype.m_body =  null;
b2Fixture.prototype.m_shape =  null;
b2Fixture.prototype.m_friction =  null;
b2Fixture.prototype.m_restitution =  null;
b2Fixture.prototype.m_proxy =  null;
b2Fixture.prototype.m_filter =  new b2FilterData();
b2Fixture.prototype.m_isSensor =  null;
b2Fixture.prototype.m_userData =  null;// aliases  
b2Fixture.prototype.create = b2Fixture.prototype.Create; 
b2Fixture.prototype.destroy = b2Fixture.prototype.Destroy; 
b2Fixture.prototype.createProxy = b2Fixture.prototype.CreateProxy; 
b2Fixture.prototype.destroyProxy = b2Fixture.prototype.DestroyProxy; 
b2Fixture.prototype.synchronize = b2Fixture.prototype.Synchronize; 
b2Fixture.prototype.getType = b2Fixture.prototype.GetType; 
b2Fixture.prototype.getShape = b2Fixture.prototype.GetShape; 
b2Fixture.prototype.setSensor = b2Fixture.prototype.SetSensor; 
b2Fixture.prototype.isSensor = b2Fixture.prototype.IsSensor; 
b2Fixture.prototype.setFilterData = b2Fixture.prototype.SetFilterData; 
b2Fixture.prototype.getFilterData = b2Fixture.prototype.GetFilterData; 
b2Fixture.prototype.getBody = b2Fixture.prototype.GetBody; 
b2Fixture.prototype.getNext = b2Fixture.prototype.GetNext; 
b2Fixture.prototype.getUserData = b2Fixture.prototype.GetUserData; 
b2Fixture.prototype.setUserData = b2Fixture.prototype.SetUserData; 
b2Fixture.prototype.testPoint = b2Fixture.prototype.TestPoint; 
b2Fixture.prototype.rayCast = b2Fixture.prototype.RayCast; 
b2Fixture.prototype.getMassData = b2Fixture.prototype.GetMassData; 
b2Fixture.prototype.setDensity = b2Fixture.prototype.SetDensity; 
b2Fixture.prototype.getDensity = b2Fixture.prototype.GetDensity; 
b2Fixture.prototype.getFriction = b2Fixture.prototype.GetFriction; 
b2Fixture.prototype.setFriction = b2Fixture.prototype.SetFriction; 
b2Fixture.prototype.getRestitution = b2Fixture.prototype.GetRestitution; 
b2Fixture.prototype.setRestitution = b2Fixture.prototype.SetRestitution; 
b2Fixture.prototype.getAABB = b2Fixture.prototype.GetAABB; 
// exports  
module.exports =  b2Fixture;
