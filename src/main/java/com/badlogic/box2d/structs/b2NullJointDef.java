package com.badlogic.box2d.structs;

import com.badlogic.gdx.jnigen.runtime.CHandler;
import com.badlogic.gdx.jnigen.runtime.pointer.Struct;
import com.badlogic.gdx.jnigen.runtime.pointer.StackElementPointer;
import com.badlogic.box2d.FFITypes;
import com.badlogic.box2d.structs.b2BodyId;
import com.badlogic.gdx.jnigen.runtime.pointer.VoidPointer;

public final class b2NullJointDef extends com.badlogic.gdx.jnigen.runtime.pointer.Struct {

    private final static int __size;

    private final static long __ffi_type;

    static {
        __ffi_type = FFITypes.getCTypeInfo(45).getFfiType();
        __size = CHandler.getSizeFromFFIType(__ffi_type);
    }

    public b2NullJointDef(long pointer, boolean freeOnGC) {
        super(pointer, freeOnGC);
    }

    public b2NullJointDef() {
        super(__size);
    }

    public long getSize() {
        return __size;
    }

    public long getFFIType() {
        return __ffi_type;
    }

    public b2NullJointDef.b2NullJointDefPointer asPointer() {
        return new b2NullJointDef.b2NullJointDefPointer(getPointer(), getsGCFreed());
    }

    public b2BodyId bodyIdA() {
        return __bodyIdA;
    }

    private static final int __bodyIdA_offset = CHandler.getOffsetForField(__ffi_type, 0);

    private final b2BodyId __bodyIdA = new b2BodyId(getPointer() + __bodyIdA_offset, false);

    public b2BodyId bodyIdB() {
        return __bodyIdB;
    }

    private static final int __bodyIdB_offset = CHandler.getOffsetForField(__ffi_type, 0);

    private final b2BodyId __bodyIdB = new b2BodyId(getPointer() + __bodyIdB_offset, false);

    public VoidPointer userData() {
        return new VoidPointer(getValue(0), false);
    }

    public void userData(VoidPointer userData) {
        setValue(userData.getPointer(), 0);
    }

    public int internalValue() {
        return (int) getValue(1);
    }

    public void internalValue(int internalValue) {
        setValue(internalValue, 1);
    }

    public static final class b2NullJointDefPointer extends StackElementPointer<b2NullJointDef> {

        public b2NullJointDefPointer(long pointer, boolean freeOnGC) {
            super(pointer, freeOnGC);
        }

        public b2NullJointDefPointer() {
            this(1, true, true);
        }

        public b2NullJointDefPointer(int count, boolean freeOnGC, boolean guard) {
            super(__size, count, freeOnGC, guard);
        }

        public b2NullJointDef.b2NullJointDefPointer guardCount(long count) {
            super.guardCount(count);
            return this;
        }

        public int getSize() {
            return __size;
        }

        protected b2NullJointDef createStackElement(long ptr, boolean freeOnGC) {
            return new b2NullJointDef(ptr, freeOnGC);
        }
    }
}
