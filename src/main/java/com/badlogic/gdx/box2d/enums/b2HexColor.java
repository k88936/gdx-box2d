package com.badlogic.gdx.box2d.enums;

import com.badlogic.gdx.jnigen.runtime.pointer.EnumPointer;
import com.badlogic.gdx.jnigen.runtime.c.CEnum;
import java.util.HashMap;

/**
 * These colors are used for debug draw and mostly match the named SVG colors.
 * See https://www.rapidtables.com/web/color/index.html
 * https://johndecember.com/html/spec/colorsvg.html
 * https://upload.wikimedia.org/wikipedia/commons/2/2b/SVG_Recognized_color_keyword_names.svg
 */
public enum b2HexColor implements CEnum {

    b2_colorBlack(0),
    b2_colorNavy(128),
    b2_colorDarkBlue(139),
    b2_colorMediumBlue(205),
    b2_colorBlue(255),
    b2_colorDarkGreen(25600),
    b2_colorGreen(32768),
    b2_colorTeal(32896),
    b2_colorDarkCyan(35723),
    b2_colorDeepSkyBlue(49151),
    b2_colorDarkTurquoise(52945),
    b2_colorMediumSpringGreen(64154),
    b2_colorLime(65280),
    b2_colorSpringGreen(65407),
    b2_colorAqua_b2_colorCyan(65535),
    b2_colorMidnightBlue(1644912),
    b2_colorDodgerBlue(2003199),
    b2_colorLightSeaGreen(2142890),
    b2_colorForestGreen(2263842),
    b2_colorSeaGreen(3050327),
    b2_colorDarkSlateGray(3100495),
    b2_colorBox2DBlue(3190463),
    b2_colorLimeGreen(3329330),
    b2_colorMediumSeaGreen(3978097),
    b2_colorTurquoise(4251856),
    b2_colorRoyalBlue(4286945),
    b2_colorSteelBlue(4620980),
    b2_colorDarkSlateBlue(4734347),
    b2_colorMediumTurquoise(4772300),
    b2_colorIndigo(4915330),
    b2_colorDarkOliveGreen(5597999),
    b2_colorCadetBlue(6266528),
    b2_colorCornflowerBlue(6591981),
    b2_colorRebeccaPurple(6697881),
    b2_colorMediumAquaMarine(6737322),
    b2_colorDimGray(6908265),
    b2_colorSlateBlue(6970061),
    b2_colorOliveDrab(7048739),
    b2_colorSlateGray(7372944),
    b2_colorLightSlateGray(7833753),
    b2_colorMediumSlateBlue(8087790),
    b2_colorLawnGreen(8190976),
    b2_colorChartreuse(8388352),
    b2_colorAquamarine(8388564),
    b2_colorMaroon(8388608),
    b2_colorPurple(8388736),
    b2_colorOlive(8421376),
    b2_colorGray(8421504),
    b2_colorSkyBlue(8900331),
    b2_colorLightSkyBlue(8900346),
    b2_colorBlueViolet(9055202),
    b2_colorDarkRed(9109504),
    b2_colorDarkMagenta(9109643),
    b2_colorSaddleBrown(9127187),
    b2_colorBox2DGreen(9226532),
    b2_colorDarkSeaGreen(9419919),
    b2_colorLightGreen(9498256),
    b2_colorMediumPurple(9662683),
    b2_colorDarkViolet(9699539),
    b2_colorPaleGreen(10025880),
    b2_colorDarkOrchid(10040012),
    b2_colorYellowGreen(10145074),
    b2_colorSienna(10506797),
    b2_colorBrown(10824234),
    b2_colorDarkGray(11119017),
    b2_colorLightBlue(11393254),
    b2_colorGreenYellow(11403055),
    b2_colorPaleTurquoise(11529966),
    b2_colorLightSteelBlue(11584734),
    b2_colorPowderBlue(11591910),
    b2_colorFireBrick(11674146),
    b2_colorDarkGoldenRod(12092939),
    b2_colorMediumOrchid(12211667),
    b2_colorRosyBrown(12357519),
    b2_colorDarkKhaki(12433259),
    b2_colorSilver(12632256),
    b2_colorMediumVioletRed(13047173),
    b2_colorIndianRed(13458524),
    b2_colorPeru(13468991),
    b2_colorChocolate(13789470),
    b2_colorTan(13808780),
    b2_colorLightGray(13882323),
    b2_colorThistle(14204888),
    b2_colorOrchid(14315734),
    b2_colorGoldenRod(14329120),
    b2_colorPaleVioletRed(14381203),
    b2_colorCrimson(14423100),
    b2_colorBox2DRed(14430514),
    b2_colorGainsboro(14474460),
    b2_colorPlum(14524637),
    b2_colorBurlywood(14596231),
    b2_colorLightCyan(14745599),
    b2_colorLavender(15132410),
    b2_colorDarkSalmon(15308410),
    b2_colorViolet(15631086),
    b2_colorPaleGoldenRod(15657130),
    b2_colorLightCoral(15761536),
    b2_colorKhaki(15787660),
    b2_colorAliceBlue(15792383),
    b2_colorHoneyDew(15794160),
    b2_colorAzure(15794175),
    b2_colorSandyBrown(16032864),
    b2_colorWheat(16113331),
    b2_colorBeige(16119260),
    b2_colorWhiteSmoke(16119285),
    b2_colorMintCream(16121850),
    b2_colorGhostWhite(16316671),
    b2_colorSalmon(16416882),
    b2_colorAntiqueWhite(16444375),
    b2_colorLinen(16445670),
    b2_colorLightGoldenRodYellow(16448210),
    b2_colorOldLace(16643558),
    b2_colorRed(16711680),
    b2_colorFuchsia_b2_colorMagenta(16711935),
    b2_colorDeepPink(16716947),
    b2_colorOrangeRed(16729344),
    b2_colorTomato(16737095),
    b2_colorHotPink(16738740),
    b2_colorCoral(16744272),
    b2_colorDarkOrange(16747520),
    b2_colorLightSalmon(16752762),
    b2_colorOrange(16753920),
    b2_colorLightPink(16758465),
    b2_colorPink(16761035),
    b2_colorGold(16766720),
    b2_colorPeachPuff(16767673),
    b2_colorNavajoWhite(16768685),
    b2_colorMoccasin(16770229),
    b2_colorBisque(16770244),
    b2_colorMistyRose(16770273),
    b2_colorBlanchedAlmond(16772045),
    b2_colorBox2DYellow(16772748),
    b2_colorPapayaWhip(16773077),
    b2_colorLavenderBlush(16773365),
    b2_colorSeaShell(16774638),
    b2_colorCornsilk(16775388),
    b2_colorLemonChiffon(16775885),
    b2_colorFloralWhite(16775920),
    b2_colorSnow(16775930),
    b2_colorYellow(16776960),
    b2_colorLightYellow(16777184),
    b2_colorIvory(16777200),
    b2_colorWhite(16777215);

    private final int index;

    b2HexColor(int index) {
        this.index = index;
    }

    public int getIndex() {
        return index;
    }

    public static b2HexColor getByIndex(int index) {
        return _values.get(index);
    }

    private final static HashMap<Integer, b2HexColor> _values = new HashMap();

    static {
        for (b2HexColor _val : values()) _values.put(_val.index, _val);
    }

    public static final class b2HexColorPointer extends EnumPointer<b2HexColor> {

        public b2HexColorPointer(long pointer, boolean freeOnGC) {
            super(pointer, freeOnGC);
        }

        public b2HexColorPointer() {
            this(1, true, true);
        }

        public b2HexColorPointer(int count, boolean freeOnGC, boolean guard) {
            super(count, freeOnGC, guard);
        }

        public b2HexColor.b2HexColorPointer guardCount(long count) {
            super.guardCount(count);
            return this;
        }

        protected b2HexColor getEnum(int index) {
            return getByIndex(index);
        }
    }
}
