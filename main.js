
var box2d = {};

box2d.Vec2         = require('./common/math/vec2');
box2d.BodyDef      = require('./dynamics/body-def');
box2d.Body         = require('./dynamics/body');
box2d.FixtureDef   = require('./dynamics/fixture-def');
box2d.Fixture      = require('./dynamics/fixture');
box2d.World        = require('./dynamics/world');
box2d.MassData     = require('./collision/shapes/mass-data');
box2d.PolygonShape = require('./collision/shapes/polygon-shape');
box2d.CircleShape  = require('./collision/shapes/circle-shape');
box2d.DebugDraw    = require('./dynamics/debug-draw');

module.exports = box2d;