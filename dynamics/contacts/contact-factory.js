// requires 
var b2ContactRegister = require('./contact-register.js'); 
var b2ContactFactory = function() {
this.__varz();
this.__constructor.apply(this, arguments);
}
b2ContactFactory.prototype.__constructor = function (allocator) {
		this.m_allocator = allocator;
		this.InitializeRegisters();
	}
b2ContactFactory.prototype.__varz = function(){
}
// static methods
// static attributes
// methods
b2ContactFactory.prototype.AddType = function (createFcn, destroyFcn, type1, type2) {
		
		
		
		this.m_registers[type1][type2].createFcn = createFcn;
		this.m_registers[type1][type2].destroyFcn = destroyFcn;
		this.m_registers[type1][type2].primary = true;
		
		if (type1 != type2)
		{
			this.m_registers[type2][type1].createFcn = createFcn;
			this.m_registers[type2][type1].destroyFcn = destroyFcn;
			this.m_registers[type2][type1].primary = false;
		}
	}
b2ContactFactory.prototype.InitializeRegisters = function () {
		this.m_registers = new Array(b2Shape.e_shapeTypeCount);
		for (var i = 0; i < b2Shape.e_shapeTypeCount; i++){
			this.m_registers[i] = new Array(b2Shape.e_shapeTypeCount);
			for (var j = 0; j < b2Shape.e_shapeTypeCount; j++){
				this.m_registers[i][j] = new b2ContactRegister();
			}
		}
		
		this.AddType(b2CircleContact.Create, b2CircleContact.Destroy, b2Shape.e_circleShape, b2Shape.e_circleShape);
		this.AddType(b2PolyAndCircleContact.Create, b2PolyAndCircleContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_circleShape);
		this.AddType(b2PolygonContact.Create, b2PolygonContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_polygonShape);
		
		this.AddType(b2EdgeAndCircleContact.Create, b2EdgeAndCircleContact.Destroy, b2Shape.e_edgeShape, b2Shape.e_circleShape);
		this.AddType(b2PolyAndEdgeContact.Create, b2PolyAndEdgeContact.Destroy, b2Shape.e_polygonShape, b2Shape.e_edgeShape);
	}
b2ContactFactory.prototype.Create = function (fixtureA, fixtureB) {
		var type1 = fixtureA.GetType();
		var type2 = fixtureB.GetType();
		
		
		
		
		var reg = this.m_registers[type1][type2];
		
		var c;
		
		if (reg.pool)
		{
			
			c = reg.pool;
			reg.pool = c.m_next;
			reg.poolCount--;
			c.Reset(fixtureA, fixtureB);
			return c;
		}
		
		var createFcn = reg.createFcn;
		if (createFcn != null)
		{
			if (reg.primary)
			{
				c = createFcn(this.m_allocator);
				c.Reset(fixtureA, fixtureB);
				return c;
			}
			else
			{
				c = createFcn(this.m_allocator);
				c.Reset(fixtureB, fixtureA);
				return c;
			}
		}
		else
		{
			return null;
		}
	}
b2ContactFactory.prototype.Destroy = function (contact) {
		if (contact.m_manifold.m_pointCount > 0)
		{
			contact.m_fixtureA.m_body.SetAwake(true);
			contact.m_fixtureB.m_body.SetAwake(true);
		}
		
		var type1 = contact.m_fixtureA.GetType();
		var type2 = contact.m_fixtureB.GetType();
		
		
		
		
		var reg = this.m_registers[type1][type2];
		
		if (true)
		{
			reg.poolCount++;
			contact.m_next = reg.pool;
			reg.pool = contact;
		}
		
		var destroyFcn = reg.destroyFcn;
		destroyFcn(contact, this.m_allocator);
	}
// attributes
b2ContactFactory.prototype.m_registers =  null;
b2ContactFactory.prototype.m_allocator =  null;// aliases  
b2ContactFactory.prototype.addType = b2ContactFactory.prototype.AddType; 
b2ContactFactory.prototype.initializeRegisters = b2ContactFactory.prototype.InitializeRegisters; 
b2ContactFactory.prototype.create = b2ContactFactory.prototype.Create; 
b2ContactFactory.prototype.destroy = b2ContactFactory.prototype.Destroy; 
// exports  
module.exports =  b2ContactFactory;
