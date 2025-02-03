package com.badlogic.gdx.box2d.structs;

import com.badlogic.gdx.jnigen.runtime.CHandler;
import com.badlogic.gdx.jnigen.runtime.pointer.Struct;
import com.badlogic.gdx.jnigen.runtime.pointer.StackElementPointer;
import com.badlogic.gdx.box2d.FFITypes;

/**
 * Surface materials allow chain shapes to have per segment surface properties.
 * @ingroup shape
 */
public final class b2SurfaceMaterial extends Struct {

    private final static int __size;

    private final static long __ffi_type;

    static {
        __ffi_type = FFITypes.getCTypeInfo(66).getFfiType();
        __size = CHandler.getSizeFromFFIType(__ffi_type);
    }

    public b2SurfaceMaterial(long pointer, boolean freeOnGC) {
        super(pointer, freeOnGC);
    }

    public b2SurfaceMaterial() {
        super(__size);
    }

    public long getSize() {
        return __size;
    }

    public long getFFIType() {
        return __ffi_type;
    }

    public b2SurfaceMaterial.b2SurfaceMaterialPointer asPointer() {
        return new b2SurfaceMaterial.b2SurfaceMaterialPointer(getPointer(), getsGCFreed());
    }

    /**
     * The Coulomb (dry) friction coefficient, usually in the range [0,1].
     */
    public float friction() {
        return (float) getValueFloat(0);
    }

    /**
     * The Coulomb (dry) friction coefficient, usually in the range [0,1].
     */
    public void friction(float friction) {
        setValue(friction, 0);
    }

    /**
     *  The coefficient of restitution (bounce) usually in the range [0,1].
     * 	 https://en.wikipedia.org/wiki/Coefficient_of_restitution
     */
    public float restitution() {
        return (float) getValueFloat(1);
    }

    /**
     *  The coefficient of restitution (bounce) usually in the range [0,1].
     * 	 https://en.wikipedia.org/wiki/Coefficient_of_restitution
     */
    public void restitution(float restitution) {
        setValue(restitution, 1);
    }

    /**
     * The rolling resistance usually in the range [0,1].
     */
    public float rollingResistance() {
        return (float) getValueFloat(2);
    }

    /**
     * The rolling resistance usually in the range [0,1].
     */
    public void rollingResistance(float rollingResistance) {
        setValue(rollingResistance, 2);
    }

    /**
     * The tangent speed for conveyor belts
     */
    public float tangentSpeed() {
        return (float) getValueFloat(3);
    }

    /**
     * The tangent speed for conveyor belts
     */
    public void tangentSpeed(float tangentSpeed) {
        setValue(tangentSpeed, 3);
    }

    /**
     *  User material identifier. This is passed with query results and to friction and restitution
     * 	 combining functions. It is not used internally.
     */
    public int material() {
        return (int) getValue(4);
    }

    /**
     *  User material identifier. This is passed with query results and to friction and restitution
     * 	 combining functions. It is not used internally.
     */
    public void material(int material) {
        setValue(material, 4);
    }

    /**
     * Custom debug draw color.
     */
    public long customColor() {
        return (long) getValue(5);
    }

    /**
     * Custom debug draw color.
     */
    public void customColor(long customColor) {
        setValue(customColor, 5);
    }

    public static final class b2SurfaceMaterialPointer extends StackElementPointer<b2SurfaceMaterial> {

        public b2SurfaceMaterialPointer(long pointer, boolean freeOnGC) {
            super(pointer, freeOnGC);
        }

        public b2SurfaceMaterialPointer() {
            this(1, true, true);
        }

        public b2SurfaceMaterialPointer(int count, boolean freeOnGC, boolean guard) {
            super(__size, count, freeOnGC, guard);
        }

        public b2SurfaceMaterial.b2SurfaceMaterialPointer guardCount(long count) {
            super.guardCount(count);
            return this;
        }

        public int getSize() {
            return __size;
        }

        protected b2SurfaceMaterial createStackElement(long ptr, boolean freeOnGC) {
            return new b2SurfaceMaterial(ptr, freeOnGC);
        }
    }
}
