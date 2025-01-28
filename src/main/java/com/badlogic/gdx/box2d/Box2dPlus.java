package com.badlogic.gdx.box2d;

import com.badlogic.gdx.box2d.structs.*;
import com.badlogic.gdx.jnigen.runtime.c.CTypeInfo;
import com.badlogic.gdx.jnigen.runtime.closure.Closure;
import com.badlogic.gdx.jnigen.runtime.closure.ClosureObject;
import com.badlogic.gdx.jnigen.runtime.ffi.JavaTypeWrapper;
import com.badlogic.gdx.math.Affine2;
import com.badlogic.gdx.math.Matrix3;
import com.badlogic.gdx.math.Vector2;

import java.util.HashMap;


public final class Box2dPlus {
    /*JNI
#include <jnigen.h>
#include <box2d/box2d.h>
static jclass illegalArgumentExceptionClass = NULL;
static jclass cxxExceptionClass = NULL;
*/

    public static void b2World_OverlapViewport(b2WorldId worldId, float lx, float ly, float ux, float uy, ClosureObject<EntityCallback> fcn) {
        b2World_OverlapViewport_internal(worldId.getPointer(), lx, ly, ux, uy, fcn.getPointer());
    }


    /*JNI
typedef bool entityCallback(long entityId);

bool viewportQuery_aux(b2ShapeId shapeId, void* context)
{
    auto callback = (entityCallback*)context;
    return callback((long)b2Body_GetUserData(b2Shape_GetBody(shapeId)));
}
*/


    private static native void b2World_OverlapViewport_internal(long worldId, float lx, float ly, float ux, float uy, long fcn);/*
    	HANDLE_JAVA_EXCEPTION_START()
    	b2AABB box;
    	box.lowerBound={lx,ly};
    	box.upperBound={ux,uy};
        b2World_OverlapAABB(*(b2WorldId*)worldId, box, b2DefaultQueryFilter(), viewportQuery_aux,(void*)fcn);
        HANDLE_JAVA_EXCEPTION_END()
    */

    public static void b2Body_SetRawUserData(b2BodyId bodyId, long userData) {
        Box2d.b2Body_SetUserData_internal(bodyId.getPointer(), userData);
    }

    public static long b2Body_GetRawUserData(b2BodyId bodyId) {
        return Box2d.b2Body_GetUserData_internal(bodyId.getPointer());
    }

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

    public static Matrix3 b2ToGDX(b2Transform b2Transform, Matrix3 gdx) {
        float c = b2Transform.q().c();
        float s = b2Transform.q().s();
        gdx.val[0] = c;
        gdx.val[1] = s;
        gdx.val[2] = 0;
        gdx.val[3] = -s;
        gdx.val[4] = c;
        gdx.val[5] = 0;
        gdx.val[6] = b2Transform.p().x();
        gdx.val[7] = b2Transform.p().y();
        gdx.val[8] = 1;
        return gdx;
    }

    public static Affine2 b2ToGDX(b2Transform b2Transform, Affine2 gdx) {
        float c = b2Transform.q().c();
        float s = b2Transform.q().s();
        gdx.m00 = c;
        gdx.m01 = -s;
        gdx.m02 = b2Transform.p().x();
        gdx.m10 = s;
        gdx.m11 = c;
        gdx.m12 = b2Transform.p().y();
        return gdx;
    }

    public static Vector2 b2ToGDX(b2Vec2 b2Vec2, Vector2 gdx) {
        gdx.x = b2Vec2.x();
        gdx.y = b2Vec2.y();
        return gdx;
    }

    public static Matrix3 b2ToGDX(b2Rot b2Rot, Matrix3 gdx) {
        float c = b2Rot.c();
        float s = b2Rot.s();
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

    public static class UserDataMappper<T> {

        private static long uid = 1;
        private final HashMap<Long, T> map = new HashMap<>();

        public void put(b2BodyId bodyId, T userData) {
            long key = b2Body_GetRawUserData(bodyId);
            if (key == 0) {
                key = uid++;
                b2Body_SetRawUserData(bodyId, key);
            }
            map.put(key, userData);
        }

        public T get(b2BodyId bodyId) {
            long key = b2Body_GetRawUserData(bodyId);
            return map.get(key);
        }

        public T remove(b2BodyId bodyId) {
            long key = b2Body_GetRawUserData(bodyId);
            b2Body_SetRawUserData(bodyId, 0);
            return map.remove(key);
        }
    }

}
