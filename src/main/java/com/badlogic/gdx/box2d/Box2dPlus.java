package com.badlogic.gdx.box2d;

import com.badlogic.gdx.box2d.structs.*;
import com.badlogic.gdx.jnigen.runtime.c.CTypeInfo;
import com.badlogic.gdx.jnigen.runtime.closure.Closure;
import com.badlogic.gdx.jnigen.runtime.closure.ClosureObject;
import com.badlogic.gdx.jnigen.runtime.ffi.JavaTypeWrapper;
import com.badlogic.gdx.math.Affine2;
import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.utils.Array;


public final class Box2dPlus {
    /*JNI
#include <jnigen.h>
#include <box2d/box2d.h>
static jclass illegalArgumentExceptionClass = NULL;
static jclass cxxExceptionClass = NULL;
*/

    //region query

    /*JNI
typedef bool entityCallback(long entityId);
*/

    /*JNI
bool overlapQuery_aux(b2ShapeId shapeId, void* context)
{
    auto callback = (entityCallback*)context;
    return callback((long long)b2Body_GetUserData(b2Shape_GetBody(shapeId)));
}
*/
    public static void b2WorldOverlapAABBbyEntity(b2WorldId worldId, float lx, float ly, float ux, float uy, ContactFilter filter, EntityCallback callback) {
        b2WorldOverlapAABBbyEntity_internal(worldId.getPointer(), lx, ly, ux, uy, filter.categoryBits, filter.maskBits, ClosureObject.fromClosure(callback).getPointer());

    }

    public static void b2WorldOverlapAABB(b2WorldId worldId, float lx, float ly, float ux, float uy,ContactFilter filter, Box2d.b2OverlapResultFcn fcn) {
        b2WorldOverlapAABB_internal(worldId.getPointer(), lx, ly, ux, uy,filter.categoryBits, filter.maskBits, ClosureObject.fromClosure(fcn).getPointer());
    }

    private static native void b2WorldOverlapAABBbyEntity_internal(long worldId, float lx, float ly, float ux, float uy,int categoryBits, int maskBits, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2AABB box;
    	box.lowerBound={lx,ly};
    	box.upperBound={ux,uy};
        b2QueryFilter filter{static_cast<uint64_t>(categoryBits), static_cast<uint64_t>(maskBits)};
        b2World_OverlapAABB(*(b2WorldId*)worldId, box, filter, overlapQuery_aux, (void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()
    */

    private static native void b2WorldOverlapAABB_internal(long worldId, float lx, float ly, float ux, float uy, int categoryBits, int maskBits, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2AABB box;
    	box.lowerBound={lx,ly};
    	box.upperBound={ux,uy};
    	b2QueryFilter filter{static_cast<uint64_t>(categoryBits), static_cast<uint64_t>(maskBits)};
        b2World_OverlapAABB(*(b2WorldId*)worldId, box, filter, (b2OverlapResultFcn* )fcn, NULL);
        HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldOverlapSquare(b2WorldId worldId, float size, Affine2 transform,ContactFilter filter, EntityCallback fcn) {
        b2WorldOverlapSquare_internal(worldId.getPointer(), size, transform.m00, transform.m10, transform.m02, transform.m12,filter.categoryBits, filter.maskBits, ClosureObject.fromClosure(fcn).getPointer());
    }

    static private native void b2WorldOverlapSquare_internal(long worldId, float size, float transform_c, float transform_s, float transform_x, float transform_y,int categoryBits, int maskBits, long fcn);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2Polygon polygon = b2MakeSquare(size);
        b2Transform transform = {b2Vec2{transform_x, transform_y}, b2Rot{transform_c, transform_s}};
        b2QueryFilter filter{static_cast<uint64_t>(categoryBits), static_cast<uint64_t>(maskBits)};
        b2World_OverlapPolygon(*(b2WorldId*)worldId, &polygon, transform, filter, overlapQuery_aux,
                               (void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldOverlapCircle(b2WorldId worldId, float size, Affine2 transform,ContactFilter filter, EntityCallback fcn) {
        b2WorldOverlapCircle_internal(worldId.getPointer(), size, transform.m02, transform.m12,filter.categoryBits, filter.maskBits, ClosureObject.fromClosure(fcn).getPointer());
    }

    static private native void b2WorldOverlapCircle_internal(long worldId, float size, float x, float y,int categoryBits, int maskBits, long fcn);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2Circle circle = {b2Vec2{x,y}, size};
        b2QueryFilter filter{static_cast<uint64_t>(categoryBits), static_cast<uint64_t>(maskBits)};
        b2World_OverlapCircle(*(b2WorldId*)worldId, &circle, b2Transform_identity, filter, overlapQuery_aux,
                               (void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldOverlapPolygon(b2WorldId worldId, b2Hull hull, Affine2 transform,ContactFilter filter, EntityCallback fcn) {
        b2WorldOverlapPolygon_internal(worldId.getPointer(), hull.getPointer(), transform.m00, transform.m10, transform.m02, transform.m12, filter.categoryBits, filter.maskBits, ClosureObject.fromClosure(fcn).getPointer());
    }

    static private native void b2WorldOverlapPolygon_internal(long worldId, long hull, float transform_c, float transform_s, float transform_x, float transform_y, int categoryBits, int maskBits, long fcn);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2Polygon polygon = b2MakePolygon((b2Hull*)hull, 0);
        b2Transform transform = {b2Vec2{transform_x, transform_y}, b2Rot{transform_c, transform_s}};
        b2QueryFilter filter{static_cast<uint64_t>(categoryBits), static_cast<uint64_t>(maskBits)};
        b2World_OverlapPolygon(*(b2WorldId*)worldId, &polygon, transform, filter, overlapQuery_aux,
                               (void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()

*/

    public static void b2WorldCastRayByOT(b2WorldId worldId, Vector2 origin, Vector2 translation, b2QueryFilter filter, Box2d.b2CastResultFcn fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, translation.x, translation.y, filter.getPointer(), ClosureObject.fromClosure(fcn).getPointer());
    }

    public static void b2WorldCastRayByOE(b2WorldId worldId, Vector2 origin, Vector2 end, b2QueryFilter filter, Box2d.b2CastResultFcn fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, end.x - origin.x, end.y - origin.y, ClosureObject.fromClosure(fcn).getPointer());
    }

    private static native void b2WorldCastRay_internal(long worldId, float originX, float originY, float translationX, float translationY, long filter, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2Vec2 origin={originX, originY};
    	b2Vec2 translation={translationX, translationY};
    	b2World_CastRay(*(b2WorldId*)worldId, origin, translation, *(b2QueryFilter*)filter, (b2CastResultFcn *)fcn, NULL);
    	HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldCastRayByOT(b2WorldId worldId, Vector2 origin, Vector2 translation, Box2d.b2CastResultFcn fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, translation.x, translation.y, ClosureObject.fromClosure(fcn).getPointer());
    }

    public static void b2WorldCastRayByOE(b2WorldId worldId, Vector2 origin, Vector2 end, Box2d.b2CastResultFcn fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, end.x - origin.x, end.y - origin.y, ClosureObject.fromClosure(fcn).getPointer());
    }

    private static native void b2WorldCastRay_internal(long worldId, float originX, float originY, float translationX, float translationY, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2Vec2 origin={originX, originY};
    	b2Vec2 translation={translationX, translationY};
    	b2World_CastRay(*(b2WorldId*)worldId, origin, translation, b2DefaultQueryFilter(), (b2CastResultFcn *)fcn, NULL);
    	HANDLE_JAVA_EXCEPTION_END()
    */

    public static boolean b2ShapeTestPoint(b2ShapeId shapeId, Vector2 point) {
        return b2ShapeTestPoint_internal(shapeId.getPointer(), point.x, point.y);
    }

    private static native boolean b2ShapeTestPoint_internal(long shapeId, float x, float y);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2Vec2 point ={x,y};
    	return (jboolean)b2Shape_TestPoint(*(b2ShapeId*)shapeId, point);
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    //endregion


    //region joint

    public static b2JointId b2ConnectBlockByWeldJoint(b2WorldId worldId, b2BodyId bodyIdA, b2BodyId bodyIdB, Vector2 localAnchorA, Vector2 localAnchorB, float referenceAngle) {
        return new b2JointId(b2ConnectBlockByWeldJoint_internal(worldId.getPointer(), bodyIdA.getPointer(), bodyIdB.getPointer(), localAnchorA.x, localAnchorA.y, localAnchorB.x, localAnchorB.y, referenceAngle), true);
    }

    static private native long b2ConnectBlockByWeldJoint_internal(long worldId, long bodyIdA, long bodyIdB, float localAnchorAx, float localAnchorAy, float localAnchorBx, float localAnchorBy, float referenceAngle);/*
    	HANDLE_JAVA_EXCEPTION_START()
        b2JointId* _ret = (b2JointId*)malloc(sizeof(b2JointId));
        b2WeldJointDef def = b2DefaultWeldJointDef();
        def.bodyIdA = *(b2BodyId*)bodyIdA;
        def.bodyIdB = *(b2BodyId*)bodyIdB;
        def.angularHertz = 0;
        def.linearHertz = 0;
        def.angularDampingRatio = 100;
        def.linearDampingRatio = 100;
        def.referenceAngle = referenceAngle;
        def.collideConnected=true;
        def.localAnchorA = b2Vec2{localAnchorAx, localAnchorAy};
        def.localAnchorB = b2Vec2{localAnchorBx, localAnchorBy};
        *_ret = b2CreateWeldJoint(*(b2WorldId*)worldId, &def);
        return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    public static b2JointId b2ConnectBlockByRevoluteJoint(b2WorldId worldId, b2BodyId bodyIdA, b2BodyId bodyIdB, Vector2 localAnchorA, Vector2 localAnchorB) {
        return new b2JointId(b2ConnectBlockByRevoluteJoint_internal(worldId.getPointer(), bodyIdA.getPointer(), bodyIdB.getPointer(), localAnchorA.x, localAnchorA.y, localAnchorB.x, localAnchorB.y), true);
    }
//
//    public static b2JointId b2ConnectBlockByRevoluteJoint(b2WorldId worldId, b2BodyId bodyIdA, b2BodyId bodyIdB, Vector2 localAnchorA, Vector2 localAnchorB, float angleLimitL, float angleLimitU, float maxTorch) {
//        b2JointId b2JointId = new b2JointId(b2ConnectBlockByRevoluteJoint_internal(worldId.getPointer(), bodyIdA.getPointer(), bodyIdB.getPointer(), localAnchorA.x, localAnchorA.y, localAnchorB.x, localAnchorB.y), true);
//        Box2d.b2RevoluteJoint_SetLimits(b2JointId, angleLimitL, angleLimitU);
//        Box2d.b2RevoluteJoint_SetMaxMotorTorque(b2JointId, maxTorch);
//        return b2JointId;
//    }

    static private native long b2ConnectBlockByRevoluteJoint_internal(long worldId, long bodyIdA, long bodyIdB, float localAnchorAx, float localAnchorAy, float localAnchorBx, float localAnchorBy);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
        def.bodyIdA = *(b2BodyId*)bodyIdA;
        def.bodyIdB = *(b2BodyId*)bodyIdB;
        def.localAnchorA = b2Vec2{localAnchorAx, localAnchorAy};
        def.localAnchorB = b2Vec2{localAnchorBx, localAnchorBy};
//        def.enableLimit = true;
//        def.lowerAngle = angleLimitL;
//        def.upperAngle = angleLimitU;
//        def.maxMotorTorque = maxTorch;
        def.enableMotor = true;
        def.hertz = 0;
        def.dampingRatio = 100;
        b2JointId* _ret = (b2JointId*)malloc(sizeof(b2JointId));
        *_ret = b2CreateRevoluteJoint(*(b2WorldId*)worldId, &def);
        return (jlong)_ret;
        HANDLE_JAVA_EXCEPTION_END()
        return 0;

    */

    //endregion

    //region create

    public static b2BodyId b2CreateCircle(b2WorldId worldId, Affine2 transform, float size, ContactFilter contactFilter) {
        return new b2BodyId(b2CreateCircle_internal(worldId.getPointer(), size, transform.m02, transform.m12, transform.m00, transform.m10, contactFilter.categoryBits, contactFilter.maskBits, contactFilter.groupIndex), true);
    }

    public static b2Hull b2ComputeHull(float[] points) {
        int count = points.length / 2;
        return new b2Hull(b2ComputeHull_internal(count, points), true);
    }

    private static native long b2ComputeHull_internal(int count, float[] data);/*
       	HANDLE_JAVA_EXCEPTION_START()
    	CHECK_AND_THROW_C_TYPE(env, int, count, 1, return 0);
        b2Vec2 points[8];
        for (int i = 0; i < count; ++i)
        {
            points[i].x = data[i * 2];
            points[i].y = data[i * 2 + 1];
        }
        b2Hull* _ret = (b2Hull*)malloc(sizeof(b2Hull));
        *_ret = b2ComputeHull(points, (int)count);
        return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;


    */

    static private native long b2CreateCircle_internal(long worldId, float size, float x, float y, float c, float s, int categoryBits, int maskBits, int groupIndex);/*

        HANDLE_JAVA_EXCEPTION_START()
        b2BodyDef def = b2DefaultBodyDef();
        def.position = {x, y};
        def.rotation = {c, s};
        def.type = b2_dynamicBody;
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.filter.categoryBits = categoryBits;
        shapeDef.filter.maskBits = maskBits;
        shapeDef.filter.groupIndex = groupIndex;
        b2Circle circle = {{0, 0}, size};
        b2BodyId* _ret = (b2BodyId*)malloc(sizeof(b2BodyId));
        *_ret = b2CreateBody(*(b2WorldId*)worldId, &def);
        b2CreateCircleShape(*_ret, &shapeDef, &circle);
        return (jlong)_ret;
        HANDLE_JAVA_EXCEPTION_END()
        return 0;
    */

    public static b2BodyId b2CreatePolygon(b2WorldId worldId, Affine2 transform, b2Hull hull, ContactFilter contactFilter) {
        return new b2BodyId(b2CreatePolygon_internal(worldId.getPointer(), transform.m02, transform.m12, transform.m00, transform.m10, hull.getPointer(), contactFilter.categoryBits, contactFilter.maskBits, contactFilter.groupIndex), true);
    }

    public static record ContactFilter(int groupIndex, int categoryBits, int maskBits) {
    }


    static private native long b2CreatePolygon_internal(long worldId, float x, float y, float c, float s, long hull, int categoryBits, int maskBits, int groupIndex);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2BodyDef def = b2DefaultBodyDef();
        def.position = {x, y};
        def.rotation = {c, s};
        def.type = b2_dynamicBody;
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.filter.categoryBits = categoryBits;
        shapeDef.filter.maskBits = maskBits;
        shapeDef.filter.groupIndex = groupIndex;
        b2Polygon polygon = b2MakePolygon((b2Hull *)hull, 0);
        b2BodyId* _ret = (b2BodyId*)malloc(sizeof(b2BodyId));
        *_ret = b2CreateBody(*(b2WorldId*)worldId, &def);
        b2CreatePolygonShape(*_ret, &shapeDef, &polygon);
        return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    public static b2BodyId b2CreateBlock(b2WorldId worldId, Affine2 transform, float size, ContactFilter contactFilter) {
        return new b2BodyId(b2CreateBlock_internal(worldId.getPointer(), size, transform.m02, transform.m12, transform.m00, transform.m10, contactFilter.categoryBits, contactFilter.maskBits, contactFilter.groupIndex), true);
    }

    static private native long b2CreateBlock_internal(long worldId, float size, float x, float y, float c, float s, int categoryBits, int maskBits, int groupIndex);/*
    	HANDLE_JAVA_EXCEPTION_START()
        b2BodyDef def = b2DefaultBodyDef();
        def.position = {x, y};
        def.rotation = {c, s};
   		def.type=b2_dynamicBody;
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.filter.categoryBits = categoryBits;
        shapeDef.filter.maskBits = maskBits;
        shapeDef.filter.groupIndex = groupIndex;
        b2Polygon polygon = b2MakeSquare(size);
        b2BodyId* _ret = (b2BodyId*)malloc(sizeof(b2BodyId));
        *_ret = b2CreateBody(*(b2WorldId*)worldId, &def);
        b2CreatePolygonShape(*_ret, &shapeDef, &polygon);
        return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    //endregion

    //region convert
    public static void b2Body_GetPosition(b2BodyId bodyId, b2Vec2 pos) {
        b2Body_GetPosition_internal(bodyId.getPointer(), pos.getPointer());
    }

    static native long b2Body_GetPosition_internal(long bodyId, long _ret);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	*(b2Vec2*)_ret = b2Body_GetPosition(*(b2BodyId*)bodyId);
    	return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    public static void b2Body_GetRotation(b2BodyId bodyId, b2Rot rot) {
        b2Body_GetRotation_internal(bodyId.getPointer(), rot.getPointer());
    }

    static native long b2Body_GetRotation_internal(long bodyId, long _ret);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	*(b2Rot*)_ret = b2Body_GetRotation(*(b2BodyId*)bodyId);
    	return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    public static void b2Body_GetTransform(b2BodyId bodyId, b2Transform transform) {
        b2Body_GetTransform_internal(bodyId.getPointer(), transform.getPointer());
    }

    static native long b2Body_GetTransform_internal(long bodyId, long _ret);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	*(b2Transform*)_ret = b2Body_GetTransform(*(b2BodyId*)bodyId);
    	return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */

    public static void b2WorldExplode(b2WorldId worldId, Vector2 position, float radius, float falloff, float power) {
        b2WorldExplode_internal(worldId.getPointer(), position.x, position.y, radius, falloff, power);
    }

    private static native void b2WorldExplode_internal(long worldId, float x, float y, float radius, float falloff, float power);/*
    	HANDLE_JAVA_EXCEPTION_START()
		auto def = b2DefaultExplosionDef();
		def.position={x,y};
		def.radius=radius;
        def.falloff=falloff;
		def.impulsePerLength=power;
    	b2World_Explode(*(b2WorldId*)worldId, &def);
    	HANDLE_JAVA_EXCEPTION_END()
    */

    public static Matrix3 b2ToGDX(b2Transform b2, Matrix3 gdx) {
        float c = b2.q().c();
        float s = b2.q().s();
        gdx.val[0] = c;
        gdx.val[1] = s;
        gdx.val[2] = 0;
        gdx.val[3] = -s;
        gdx.val[4] = c;
        gdx.val[5] = 0;
        gdx.val[6] = b2.p().x();
        gdx.val[7] = b2.p().y();
        gdx.val[8] = 1;
        return gdx;
    }

    public static Affine2 b2ToGDX(b2Transform b2, Affine2 gdx) {
        float c = b2.q().c();
        float s = b2.q().s();
        gdx.m00 = c;
        gdx.m01 = -s;
        gdx.m02 = b2.p().x();
        gdx.m10 = s;
        gdx.m11 = c;
        gdx.m12 = b2.p().y();
        return gdx;
    }

    public static b2Transform GDXTob2(Affine2 gdx, b2Transform b2) {
        b2.p().x(gdx.m02);
        b2.p().y(gdx.m12);
        b2.q().c(gdx.m00);
        b2.q().s(gdx.m10);
        return b2;

    }

    public static Vector2 b2ToGDX(b2Vec2 b2, Vector2 gdx) {
        gdx.x = b2.x();
        gdx.y = b2.y();
        return gdx;
    }

    public static b2Vec2 GDXTob2(Vector2 gdx, b2Vec2 b2) {
        b2.x(gdx.x);
        b2.y(gdx.y);
        return b2;
    }

    public static Matrix3 b2ToGDX(b2Rot b2, Matrix3 gdx) {
        float c = b2.c();
        float s = b2.s();
        gdx.val[0] = c;
        gdx.val[1] = s;
        gdx.val[2] = 0;
        gdx.val[3] = -s;
        gdx.val[4] = c;
        gdx.val[5] = 0;
        gdx.val[6] = 0;
        gdx.val[7] = 0;
        gdx.val[8] = 1;
        return gdx;
    }

    public static Affine2 b2ToGDX(b2Rot b2Rot, Affine2 gdx) {
        float c = b2Rot.c();
        float s = b2Rot.s();
        gdx.m00 = c;
        gdx.m01 = -s;
        gdx.m02 = 0;
        gdx.m10 = s;
        gdx.m11 = c;
        gdx.m12 = 0;
        return gdx;
    }

    public static b2Rot GDXTob2(Affine2 gdx, b2Rot b2) {
        b2.c(gdx.m00);
        b2.s(gdx.m10);
        return b2;
    }


    public static b2Vec2 GDXTob2(Affine2 gdx, b2Vec2 b2) {
        b2.x(gdx.m02);
        b2.y(gdx.m12);
        return b2;
    }
    //endregion

    public static void b2BodySetRawUserData(b2BodyId bodyId, long userData) {
        Box2d.b2Body_SetUserData_internal(bodyId.getPointer(), userData);
    }
    //region userdata

    public static long b2BodyGetRawUserData(b2BodyId bodyId) {
        return Box2d.b2Body_GetUserData_internal(bodyId.getPointer());
    }

    public interface EntityCallback extends Closure {

        CTypeInfo[] __ffi_cache = new CTypeInfo[]{FFITypes.getCTypeInfo(0), FFITypes.getCTypeInfo(4)};

        boolean b2OverlapResultFcn_call(long entity);

        default CTypeInfo[] functionSignature() {
            return __ffi_cache;
        }

        default void invoke(JavaTypeWrapper[] parameters, JavaTypeWrapper returnType) {
            returnType.setValue(b2OverlapResultFcn_call(parameters[0].asLong()));
        }
    }

    public static class BodyUserDataMapper<T extends Id, U> {


        private final Array<Array<U>> content = new Array<>();

        public void put(T id, U userData) {
            int i = id.index1();
            int w = id.world0();
            if (w > content.size) {
                content.ensureCapacity(2 * w);
            }
            Array<U> ts = content.get(w);
            if (ts == null) {
                ts = new Array<>();
                content.set(w, ts);
            }

            ts.set(i, userData);
        }

        public U get(T id) {
            int i = id.index1();
            int w = id.world0();
//            if (w > content.size) {
//                return null;
//            }
            Array<U> ts = content.get(w);
//            if (ts == null) {
//                return null;
//            }
            return ts.get(i);
        }

        public U remove(T id) {
            int i = id.index1();
            int w = id.world0();
//            if (w > content.size) {
//                return null;
//            }
            Array<U> ts = content.get(w);
//            if (ts == null) {
//                return null;
//            }
            return ts.removeIndex(i);
        }
    }

    //endregion
}
