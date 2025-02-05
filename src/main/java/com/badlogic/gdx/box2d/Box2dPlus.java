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
    public static void b2WorldOverlapAABBbyEntity(b2WorldId worldId, float lx, float ly, float ux, float uy, ClosureObject<EntityCallback> fcn) {
        b2WorldOverlapAABBbyEntity_internal(worldId.getPointer(), lx, ly, ux, uy, fcn.getPointer());

    }

    public static void b2WorldOverlapAABB(b2WorldId worldId, float lx, float ly, float ux, float uy, ClosureObject<Box2d.b2OverlapResultFcn> fcn) {
        b2WorldOverlapAABB_internal(worldId.getPointer(), lx, ly, ux, uy, fcn.getPointer());
    }

    private static native void b2WorldOverlapAABBbyEntity_internal(long worldId, float lx, float ly, float ux, float uy, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2AABB box;
    	box.lowerBound={lx,ly};
    	box.upperBound={ux,uy};
        b2World_OverlapAABB(*(b2WorldId*)worldId, box, b2DefaultQueryFilter(), overlapQuery_aux, (void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()
    */

    private static native void b2WorldOverlapAABB_internal(long worldId, float lx, float ly, float ux, float uy, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2AABB box;
    	box.lowerBound={lx,ly};
    	box.upperBound={ux,uy};
        b2World_OverlapAABB(*(b2WorldId*)worldId, box, b2DefaultQueryFilter(), (b2OverlapResultFcn* )fcn, NULL);
        HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldOverlapSquare(b2WorldId worldId, float size, Affine2 transform, ClosureObject<EntityCallback> fcn) {
        b2WorldOverlapSquare_internal(worldId.getPointer(), size, transform.m00, transform.m10, transform.m02, transform.m12, fcn.getPointer());
    }

    static private native void b2WorldOverlapSquare_internal(long worldId, float size, float transform_c, float transform_s, float transform_x, float transform_y, long fcn);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2Polygon polygon = b2MakeSquare(size);
        b2Transform transform = {b2Vec2{transform_x, transform_y}, b2Rot{transform_c, transform_s}};
        b2World_OverlapPolygon(*(b2WorldId*)worldId, &polygon, transform, b2DefaultQueryFilter(), overlapQuery_aux,
                               (void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldCastRayByOT(b2WorldId worldId, Vector2 origin, Vector2 translation, b2QueryFilter filter, ClosureObject<Box2d.b2CastResultFcn> fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, translation.x, translation.y, filter.getPointer(), fcn.getPointer());
    }

    public static void b2WorldCastRayByOE(b2WorldId worldId, Vector2 origin, Vector2 end, b2QueryFilter filter, ClosureObject<Box2d.b2CastResultFcn> fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, end.x - origin.x, end.y - origin.y, fcn.getPointer());
    }

    private static native void b2WorldCastRay_internal(long worldId, float originX, float originY, float translationX, float translationY, long filter, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2Vec2 origin={originX, originY};
    	b2Vec2 translation={translationX, translationY};
    	b2World_CastRay(*(b2WorldId*)worldId, origin, translation, *(b2QueryFilter*)filter, (b2CastResultFcn *)fcn, NULL);
    	HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2WorldCastRayByOT(b2WorldId worldId, Vector2 origin, Vector2 translation, ClosureObject<Box2d.b2CastResultFcn> fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, translation.x, translation.y, fcn.getPointer());
    }

    public static void b2WorldCastRayByOE(b2WorldId worldId, Vector2 origin, Vector2 end, ClosureObject<Box2d.b2CastResultFcn> fcn) {
        b2WorldCastRay_internal(worldId.getPointer(), origin.x, origin.y, end.x - origin.x, end.y - origin.y, fcn.getPointer());
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

    public static void b2BodySetRawUserData(b2BodyId bodyId, long userData) {
        Box2d.b2Body_SetUserData_internal(bodyId.getPointer(), userData);
    }

    public static long b2BodyGetRawUserData(b2BodyId bodyId) {
        return Box2d.b2Body_GetUserData_internal(bodyId.getPointer());
    }

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

    public static b2JointId b2ConnectBlockByRevoluteJoint(b2WorldId worldId, b2BodyId bodyIdA, b2BodyId bodyIdB, Vector2 localAnchor, float angleLimitL, float angleLimitU, float maxTorch) {
        return new b2JointId(b2ConnectBlockByRevoluteJoint_internal(worldId.getPointer(), bodyIdA.getPointer(), bodyIdB.getPointer(), localAnchor.x, localAnchor.y, localAnchor.x, localAnchor.y, angleLimitL, angleLimitU, maxTorch), true);
    }

    static private native long b2ConnectBlockByRevoluteJoint_internal(long worldId, long bodyIdA, long bodyIdB, float localAnchorAx, float localAnchorAy, float localAnchorBx, float localAnchorBy, float angleLimitL, float angleLimitU, float maxTorch);/*
        HANDLE_JAVA_EXCEPTION_START()
        b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
        def.bodyIdA = *(b2BodyId*)bodyIdA;
        def.bodyIdB = *(b2BodyId*)bodyIdB;
        def.localAnchorA = b2Vec2{localAnchorAx, localAnchorAy};
        def.localAnchorB = b2Vec2{localAnchorBx, localAnchorBy};
        def.enableLimit = true;
        def.lowerAngle = angleLimitL;
        def.upperAngle = angleLimitU;
        def.maxMotorTorque = maxTorch;
        def.enableMotor = true;
        def.hertz = 0;
        def.dampingRatio = 100;
        b2JointId* _ret = (b2JointId*)malloc(sizeof(b2JointId));
        *_ret = b2CreateRevoluteJoint(*(b2WorldId*)worldId, &def);
        return (jlong)_ret;
        HANDLE_JAVA_EXCEPTION_END()
        return 0;

    */

    public static b2BodyId b2CreateBlock(b2WorldId worldId, Affine2 transform, float size) {
        return new b2BodyId(b2CreateBlock_internal(worldId.getPointer(), size, transform.m02, transform.m12, transform.m00, transform.m10), true);
    }

    static private native long b2CreateBlock_internal(long worldId, float size, float x, float y, float c, float s);/*
    	HANDLE_JAVA_EXCEPTION_START()
        b2BodyDef def = b2DefaultBodyDef();
        def.position = {x, y};
        def.rotation = {c, s};
        b2ShapeDef shapeDef = b2DefaultShapeDef();
        b2Polygon polygon = b2MakeSquare(size);
        b2BodyId* _ret = (b2BodyId*)malloc(sizeof(b2BodyId));
        *_ret = b2CreateBody(*(b2WorldId*)worldId, &def);
        b2CreatePolygonShape(*_ret, &shapeDef, &polygon);
        return (jlong)_ret;
    	HANDLE_JAVA_EXCEPTION_END()
    	return 0;
    */


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

}
