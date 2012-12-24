
var box2d = {};

box2d.Vec2         = require('../lib/common/math/vec2');
box2d.BodyDef      = require('../lib/dynamics/body-def');
box2d.Body         = require('../lib/dynamics/body');
box2d.FixtureDef   = require('../lib/dynamics/fixture-def');
box2d.Fixture      = require('../lib/dynamics/fixture');
box2d.World        = require('../lib/dynamics/world');
box2d.MassData     = require('../lib/collision/shapes/mass-data');
box2d.PolygonShape = require('../lib/collision/shapes/polygon-shape');
box2d.CircleShape  = require('../lib/collision/shapes/circle-shape');
box2d.DebugDraw    = require('../lib/dynamics/debug-draw');

module.export = box2d;