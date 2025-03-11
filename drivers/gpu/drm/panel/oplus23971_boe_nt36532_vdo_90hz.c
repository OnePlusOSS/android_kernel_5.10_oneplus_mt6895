// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <linux/of_graph.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include <linux/regulator/consumer.h>

#include <soc/oplus/system/boot_mode.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#endif
#include "ktz8866.h"
#include "../mediatek/mediatek_v2/mtk_corner_pattern/mtk_data_hw_roundedpattern.h"

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#endif
#else
extern int get_boot_mode(void);
#endif

/* enable this to check panel self -bist pattern */
/* #define PANEL_BIST_PATTERN */
/****************TPS65132***********/
//#include "lcm_i2c.h"

//static char bl_tb0[] = { 0x51, 0x0f, 0xff };
extern int lcd_bl_set_led_brightness(int value);
extern int lcd_bl_write_byte(unsigned char addr, unsigned char value);
//TP reset
//extern void (himax_rst_gpio_high)(void);
#ifdef HIMAX_WAKE_UP
extern uint8_t wake_flag_drm;
#endif

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
static int mode;
#endif
extern int shut_down_flag;

/* TP gesture mode */
static bool tp_gusture_mode = false;
static bool esd_flag = false;
static unsigned int bl_last_level = 0;

/* TP define */
#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_CS_ON  0x19

//TO DO: You have to do that remove macro BYPASSI2C and solve build error
//otherwise voltage will be unstable
#define  M_DELAY(n) usleep_range(n*1000, n*1000+100)
#define  U_DELAY(n) usleep_range(n, n+10)

enum CMD_TYPE {
	TYPE_GERNERIC,
	TYPE_PPS,
	TYPE_DCS,
	TYPE_MDELAY,
	TYPE_UDELAY
};

struct LCM_setting_table_by_type {
	unsigned int type;
	unsigned char count;
	unsigned char para_list[64];
};

struct boe {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *v1v8_enable_gpio;
	struct gpio_desc *bias_pos;
	struct gpio_desc *bias_neg;
	struct gpio_desc *bias_en;
	struct gpio_desc *esd_te_gpio;
	bool prepared;
	bool enabled;

	unsigned int gate_ic;

	int error;
};

static inline struct boe *panel_to_boe(struct drm_panel *panel)
{
	return container_of(panel, struct boe, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int boe_dcs_read(struct boe *ctx, u8 cmd, void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;

	if (ctx->error < 0)
		return 0;

	ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
			 cmd);
		ctx->error = ret;
	}

	return ret;
}

static void boe_panel_get_data(struct boe *ctx)
{
	u8 buffer[3] = { 0 };
	static int ret;

	pr_info("%s+\n", __func__);

	if (ret == 0) {
		ret = boe_dcs_read(ctx, 0x0A, buffer, 1);
		pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
		dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
			ret, buffer[0] | (buffer[1] << 8));
	}
}
#endif

static void boe_init_write_by_type(struct boe *ctx, struct LCM_setting_table_by_type *setting)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret = 0;

	if (ctx->error < 0) {
		pr_err("[lcd_info] %s: ctx->error=%d, LINE=%d\n",__func__, ctx->error, __LINE__);
		return;
	}

	switch(setting->type) {
		case TYPE_GERNERIC:
			ret = mipi_dsi_generic_write(dsi, setting->para_list, setting->count);
			break;
		case TYPE_PPS:
			ret = mipi_dsi_picture_parameter_set(dsi, (struct drm_dsc_picture_parameter_set *)(setting->para_list));
			break;
		case TYPE_DCS:
			ret = mipi_dsi_dcs_write_buffer(dsi, setting->para_list, setting->count);
			break;
		case TYPE_MDELAY:
			M_DELAY(setting->count);
			break;
		case TYPE_UDELAY:
			U_DELAY(setting->count);
			break;
		default:
			pr_err("[lcd_info] %s: type=%d is invalid\n",__func__, setting->type);
			break;
	}
	if (ret < 0) {
		pr_err("[lcd_info] error %zd writing seq: %ph\n", ret, *setting);
		ctx->error = ret;
	}
}

static unsigned int backlight_mapping_buf[] = {
	   0,  284,  300,  316,  332,  348,  364,  380,  396,  412,  428,  444,  460,  476,  492,  508,  524,  540,  555,  570,
	 585,  593,  600,  602,  605,  607,  609,  611,  613,  615,  617,  619,  621,  623,  624,  627,  629,  630,  632,  635,
	 636,  638,  640,  642,  644,  646,  647,  650,  651,  653,  655,  657,  659,  660,  662,  663,  666,  667,  669,  671,
	 673,  675,  676,  678,  680,  682,  683,  685,  686,  689,  690,  691,  693,  694,  697,  698,  700,  701,  704,  705,
	 706,  708,  709,  712,  713,  714,  716,  717,  720,  721,  722,  724,  726,  727,  729,  730,  731,  734,  735,  736,
	 738,  739,  741,  743,  744,  745,  747,  749,  750,  752,  753,  754,  755,  758,  759,  760,  761,  764,  765,  766,
	 767,  769,  770,  772,  773,  775,  776,  777,  778,  781,  782,  783,  784,  785,  788,  789,  790,  791,  792,  795,
	 796,  797,  798,  799,  800,  803,  804,  805,  806,  807,  808,  810,  812,  813,  814,  815,  816,  818,  819,  821,
	 822,  823,  824,  826,  827,  828,  829,  830,  833,  834,  835,  836,  837,  838,  839,  841,  842,  843,  844,  846,
	 847,  849,  850,  851,  852,  853,  854,  856,  857,  858,  859,  860,  861,  862,  864,  865,  866,  867,  868,  869,
	 872,  873,  874,  875,  876,  877,  879,  880,  881,  882,  883,  884,  885,  887,  888,  889,  890,  891,  892,  893,
	 895,  896,  897,  898,  899,  899,  900,  902,  903,  904,  905,  906,  907,  908,  910,  911,  912,  913,  914,  915,
	 916,  918,  919,  920,  921,  922,  923,  923,  925,  926,  927,  928,  929,  930,  931,  933,  934,  935,  936,  937,
	 937,  938,  939,  941,  942,  943,  944,  945,  946,  948,  949,  949,  950,  951,  952,  953,  954,  956,  957,  958,
	 958,  959,  960,  961,  962,  964,  965,  966,  966,  967,  968,  969,  971,  972,  973,  974,  974,  975,  976,  977,
	 979,  980,  981,  981,  982,  983,  984,  985,  987,  988,  988,  989,  990,  991,  992,  994,  994,  995,  996,  997,
	 998,  999,  999, 1000, 1002, 1003, 1004, 1005, 1005, 1006, 1007, 1008, 1010, 1010, 1011, 1012, 1013, 1014, 1015, 1015,
	1017, 1018, 1019, 1020, 1020, 1021, 1022, 1023, 1025, 1025, 1026, 1027, 1028, 1029, 1029, 1030, 1031, 1033, 1034, 1034,
	1035, 1036, 1037, 1037, 1038, 1040, 1041, 1042, 1042, 1043, 1044, 1045, 1045, 1046, 1048, 1049, 1050, 1050, 1051, 1052,
	1053, 1053, 1054, 1056, 1057, 1057, 1058, 1059, 1060, 1061, 1061, 1063, 1064, 1065, 1065, 1066, 1067, 1068, 1068, 1069,
	1071, 1072, 1072, 1073, 1074, 1075, 1075, 1076, 1077, 1079, 1079, 1080, 1081, 1082, 1082, 1083, 1084, 1084, 1086, 1087,
	1088, 1088, 1089, 1090, 1091, 1091, 1092, 1094, 1095, 1095, 1096, 1097, 1097, 1098, 1099, 1100, 1100, 1102, 1103, 1104,
	1104, 1105, 1106, 1106, 1107, 1109, 1110, 1110, 1111, 1112, 1112, 1113, 1114, 1115, 1115, 1117, 1118, 1118, 1119, 1120,
	1120, 1121, 1122, 1123, 1123, 1125, 1126, 1126, 1127, 1128, 1128, 1129, 1130, 1132, 1132, 1133, 1134, 1134, 1135, 1136,
	1136, 1137, 1138, 1138, 1140, 1141, 1142, 1142, 1143, 1144, 1144, 1145, 1146, 1146, 1148, 1149, 1149, 1150, 1151, 1151,
	1152, 1153, 1153, 1155, 1156, 1156, 1157, 1158, 1159, 1159, 1160, 1161, 1161, 1163, 1164, 1164, 1165, 1166, 1166, 1167,
	1168, 1168, 1169, 1171, 1171, 1172, 1173, 1173, 1174, 1175, 1175, 1176, 1176, 1178, 1179, 1179, 1180, 1181, 1181, 1182,
	1183, 1183, 1184, 1186, 1186, 1187, 1188, 1188, 1189, 1190, 1190, 1191, 1192, 1192, 1194, 1195, 1195, 1196, 1196, 1197,
	1198, 1198, 1199, 1201, 1201, 1202, 1203, 1203, 1204, 1205, 1205, 1206, 1206, 1207, 1209, 1209, 1210, 1211, 1211, 1212,
	1213, 1213, 1214, 1214, 1215, 1217, 1217, 1218, 1219, 1219, 1220, 1221, 1221, 1222, 1222, 1224, 1225, 1225, 1226, 1227,
	1227, 1228, 1228, 1229, 1230, 1230, 1232, 1233, 1233, 1234, 1234, 1235, 1236, 1236, 1237, 1238, 1238, 1240, 1240, 1241,
	1242, 1242, 1243, 1243, 1244, 1245, 1245, 1247, 1248, 1248, 1249, 1249, 1250, 1251, 1251, 1252, 1252, 1253, 1255, 1255,
	1256, 1256, 1257, 1258, 1258, 1259, 1260, 1260, 1261, 1261, 1263, 1264, 1264, 1265, 1265, 1266, 1267, 1267, 1268, 1268,
	1270, 1271, 1271, 1272, 1272, 1273, 1274, 1274, 1275, 1275, 1276, 1278, 1278, 1279, 1279, 1280, 1281, 1281, 1282, 1282,
	1283, 1284, 1284, 1286, 1286, 1287, 1288, 1288, 1289, 1289, 1290, 1290, 1291, 1293, 1293, 1294, 1294, 1295, 1296, 1296,
	1297, 1297, 1298, 1299, 1299, 1301, 1301, 1302, 1302, 1303, 1304, 1304, 1305, 1305, 1306, 1307, 1307, 1309, 1309, 1310,
	1310, 1311, 1312, 1312, 1313, 1313, 1314, 1314, 1316, 1317, 1317, 1318, 1318, 1319, 1320, 1320, 1321, 1321, 1322, 1322,
	1324, 1325, 1325, 1326, 1326, 1327, 1327, 1328, 1329, 1329, 1330, 1330, 1332, 1332, 1333, 1334, 1334, 1335, 1335, 1336,
	1336, 1337, 1339, 1339, 1340, 1340, 1341, 1341, 1342, 1343, 1343, 1344, 1344, 1345, 1345, 1347, 1347, 1348, 1349, 1349,
	1350, 1350, 1351, 1351, 1352, 1353, 1353, 1355, 1355, 1356, 1356, 1357, 1357, 1358, 1359, 1359, 1360, 1360, 1362, 1362,
	1363, 1363, 1364, 1365, 1365, 1366, 1366, 1367, 1367, 1368, 1368, 1370, 1371, 1371, 1372, 1372, 1373, 1373, 1374, 1374,
	1375, 1376, 1376, 1378, 1378, 1379, 1379, 1380, 1380, 1381, 1382, 1382, 1383, 1383, 1385, 1385, 1386, 1386, 1387, 1387,
	1388, 1389, 1389, 1390, 1390, 1391, 1391, 1393, 1393, 1394, 1394, 1395, 1396, 1396, 1397, 1397, 1398, 1398, 1400, 1400,
	1401, 1401, 1402, 1403, 1403, 1404, 1404, 1405, 1405, 1406, 1406, 1408, 1408, 1409, 1409, 1410, 1411, 1411, 1412, 1412,
	1413, 1413, 1414, 1414, 1416, 1416, 1417, 1417, 1418, 1419, 1419, 1420, 1420, 1421, 1421, 1423, 1423, 1424, 1424, 1425,
	1425, 1426, 1427, 1427, 1428, 1428, 1429, 1429, 1431, 1431, 1432, 1432, 1433, 1433, 1434, 1434, 1435, 1435, 1436, 1437,
	1437, 1439, 1439, 1440, 1440, 1441, 1441, 1442, 1442, 1443, 1443, 1444, 1444, 1446, 1446, 1447, 1448, 1448, 1449, 1449,
	1450, 1450, 1451, 1451, 1452, 1452, 1454, 1454, 1455, 1455, 1456, 1456, 1457, 1457, 1458, 1458, 1459, 1460, 1460, 1462,
	1462, 1463, 1463, 1464, 1464, 1465, 1465, 1466, 1466, 1467, 1467, 1469, 1469, 1470, 1470, 1471, 1471, 1472, 1472, 1473,
	1474, 1474, 1475, 1475, 1477, 1477, 1478, 1478, 1479, 1479, 1480, 1480, 1481, 1481, 1482, 1482, 1483, 1483, 1485, 1485,
	1486, 1486, 1487, 1487, 1488, 1488, 1489, 1489, 1490, 1490, 1492, 1493, 1493, 1494, 1494, 1495, 1495, 1496, 1496, 1497,
	1497, 1498, 1498, 1500, 1500, 1501, 1501, 1502, 1502, 1503, 1503, 1504, 1504, 1505, 1505, 1506, 1506, 1508, 1508, 1509,
	1509, 1510, 1510, 1511, 1511, 1512, 1512, 1513, 1513, 1515, 1515, 1516, 1516, 1517, 1517, 1518, 1518, 1519, 1519, 1520,
	1520, 1521, 1521, 1523, 1523, 1524, 1525, 1525, 1526, 1526, 1527, 1527, 1528, 1528, 1529, 1529, 1531, 1531, 1532, 1532,
	1533, 1533, 1534, 1534, 1535, 1535, 1536, 1536, 1538, 1538, 1539, 1539, 1540, 1540, 1541, 1541, 1542, 1542, 1543, 1543,
	1544, 1544, 1546, 1546, 1547, 1547, 1548, 1548, 1549, 1549, 1550, 1550, 1551, 1551, 1552, 1552, 1554, 1554, 1555, 1555,
	1556, 1556, 1557, 1557, 1558, 1558, 1559, 1559, 1561, 1561, 1562, 1562, 1563, 1563, 1564, 1564, 1564, 1565, 1565, 1566,
	1566, 1567, 1567, 1569, 1569, 1570, 1570, 1571, 1571, 1572, 1572, 1573, 1573, 1574, 1574, 1575, 1575, 1577, 1577, 1578,
	1578, 1579, 1579, 1580, 1580, 1581, 1581, 1582, 1582, 1584, 1584, 1585, 1585, 1586, 1586, 1587, 1587, 1588, 1588, 1589,
	1589, 1590, 1590, 1592, 1592, 1593, 1593, 1594, 1594, 1595, 1595, 1596, 1596, 1597, 1597, 1598, 1598, 1598, 1600, 1600,
	1601, 1601, 1602, 1602, 1603, 1603, 1604, 1604, 1605, 1605, 1607, 1607, 1608, 1608, 1609, 1609, 1610, 1610, 1611, 1611,
	1612, 1612, 1613, 1613, 1615, 1615, 1616, 1616, 1617, 1617, 1618, 1618, 1618, 1619, 1619, 1620, 1620, 1621, 1621, 1623,
	1623, 1624, 1624, 1625, 1625, 1626, 1626, 1627, 1627, 1628, 1628, 1630, 1630, 1631, 1631, 1632, 1632, 1633, 1633, 1634,
	1634, 1634, 1635, 1635, 1636, 1636, 1638, 1638, 1639, 1639, 1640, 1640, 1641, 1641, 1642, 1642, 1643, 1643, 1644, 1644,
	1646, 1646, 1647, 1647, 1648, 1648, 1648, 1649, 1649, 1650, 1650, 1651, 1651, 1653, 1653, 1654, 1654, 1655, 1655, 1656,
	1656, 1657, 1657, 1658, 1658, 1659, 1659, 1661, 1661, 1661, 1662, 1662, 1663, 1663, 1664, 1664, 1665, 1665, 1666, 1666,
	1667, 1667, 1669, 1669, 1670, 1670, 1671, 1671, 1672, 1672, 1672, 1673, 1673, 1674, 1674, 1676, 1676, 1677, 1677, 1678,
	1678, 1679, 1679, 1680, 1680, 1681, 1681, 1682, 1682, 1682, 1684, 1684, 1685, 1685, 1686, 1686, 1687, 1687, 1688, 1688,
	1689, 1689, 1690, 1690, 1692, 1692, 1692, 1693, 1693, 1694, 1694, 1695, 1695, 1696, 1696, 1697, 1697, 1699, 1699, 1700,
	1700, 1701, 1701, 1702, 1702, 1702, 1703, 1703, 1704, 1704, 1705, 1705, 1707, 1707, 1708, 1708, 1709, 1709, 1710, 1710,
	1710, 1711, 1711, 1712, 1712, 1713, 1713, 1715, 1715, 1716, 1716, 1717, 1717, 1718, 1718, 1719, 1719, 1719, 1720, 1720,
	1722, 1722, 1723, 1723, 1724, 1724, 1725, 1725, 1726, 1726, 1727, 1727, 1727, 1728, 1728, 1730, 1730, 1731, 1731, 1732,
	1732, 1733, 1733, 1734, 1734, 1735, 1735, 1735, 1736, 1736, 1738, 1738, 1739, 1739, 1740, 1740, 1741, 1741, 1742, 1742,
	1742, 1743, 1743, 1745, 1745, 1746, 1746, 1747, 1747, 1748, 1748, 1749, 1749, 1750, 1750, 1750, 1751, 1751, 1753, 1753,
	1754, 1754, 1755, 1755, 1756, 1756, 1757, 1757, 1757, 1758, 1758, 1759, 1759, 1761, 1761, 1762, 1762, 1763, 1763, 1764,
	1764, 1764, 1765, 1765, 1766, 1766, 1768, 1768, 1769, 1769, 1770, 1770, 1771, 1771, 1771, 1772, 1772, 1773, 1773, 1774,
	1774, 1776, 1776, 1777, 1777, 1778, 1778, 1778, 1779, 1779, 1780, 1780, 1781, 1781, 1782, 1782, 1784, 1784, 1785, 1785,
	1785, 1786, 1786, 1787, 1787, 1788, 1788, 1789, 1789, 1791, 1791, 1791, 1792, 1792, 1793, 1793, 1794, 1794, 1795, 1795,
	1796, 1796, 1797, 1797, 1797, 1799, 1799, 1800, 1800, 1801, 1801, 1802, 1802, 1803, 1803, 1803, 1804, 1804, 1805, 1805,
	1807, 1807, 1808, 1808, 1809, 1809, 1810, 1810, 1810, 1811, 1811, 1812, 1812, 1814, 1814, 1815, 1815, 1816, 1816, 1816,
	1817, 1817, 1818, 1818, 1819, 1819, 1820, 1820, 1822, 1822, 1822, 1823, 1823, 1824, 1824, 1825, 1825, 1826, 1826, 1827,
	1827, 1827, 1828, 1828, 1830, 1830, 1831, 1831, 1832, 1832, 1833, 1833, 1833, 1834, 1834, 1835, 1835, 1837, 1837, 1838,
	1838, 1839, 1839, 1839, 1840, 1840, 1841, 1841, 1842, 1842, 1843, 1843, 1845, 1845, 1845, 1846, 1846, 1847, 1847, 1848,
	1848, 1849, 1849, 1850, 1850, 1850, 1851, 1851, 1853, 1853, 1854, 1854, 1855, 1855, 1856, 1856, 1856, 1857, 1857, 1858,
	1858, 1860, 1860, 1861, 1861, 1861, 1862, 1862, 1863, 1863, 1864, 1864, 1865, 1865, 1866, 1866, 1866, 1868, 1868, 1869,
	1869, 1870, 1870, 1871, 1871, 1872, 1872, 1872, 1873, 1873, 1874, 1874, 1876, 1876, 1877, 1877, 1877, 1878, 1878, 1879,
	1879, 1880, 1880, 1881, 1881, 1883, 1883, 1883, 1884, 1884, 1885, 1885, 1886, 1886, 1887, 1887, 1887, 1888, 1888, 1889,
	1889, 1891, 1891, 1892, 1892, 1892, 1893, 1893, 1894, 1894, 1895, 1895, 1896, 1896, 1897, 1897, 1897, 1899, 1899, 1900,
	1900, 1901, 1901, 1902, 1902, 1902, 1903, 1903, 1904, 1904, 1906, 1906, 1907, 1907, 1908, 1908, 1908, 1909, 1909, 1910,
	1910, 1911, 1911, 1912, 1912, 1912, 1914, 1914, 1915, 1915, 1916, 1916, 1917, 1917, 1917, 1918, 1918, 1919, 1919, 1920,
	1920, 1922, 1922, 1922, 1923, 1923, 1924, 1924, 1925, 1925, 1926, 1926, 1926, 1927, 1927, 1929, 1929, 1930, 1930, 1931,
	1931, 1932, 1932, 1932, 1933, 1933, 1934, 1934, 1935, 1935, 1937, 1937, 1937, 1938, 1938, 1939, 1939, 1940, 1940, 1941,
	1941, 1941, 1942, 1942, 1943, 1943, 1945, 1945, 1946, 1946, 1946, 1947, 1947, 1948, 1948, 1949, 1949, 1950, 1950, 1950,
	1952, 1952, 1953, 1953, 1954, 1954, 1955, 1955, 1955, 1956, 1956, 1957, 1957, 1958, 1958, 1960, 1960, 1960, 1961, 1961,
	1962, 1962, 1963, 1963, 1964, 1964, 1964, 1965, 1965, 1966, 1966, 1968, 1968, 1969, 1969, 1969, 1970, 1970, 1971, 1971,
	1972, 1972, 1973, 1973, 1973, 1975, 1975, 1976, 1976, 1977, 1977, 1977, 1978, 1978, 1979, 1979, 1980, 1980, 1981, 1981,
	1981, 1983, 1983, 1984, 1984, 1985, 1985, 1986, 1986, 1986, 1987, 1987, 1988, 1988, 1989, 1989, 1991, 1991, 1991, 1992,
	1992, 1993, 1993, 1994, 1994, 1995, 1995, 1995, 1996, 1996, 1998, 1998, 1999, 1999, 2000, 2000, 2000, 2001, 2001, 2002,
	2002, 2003, 2003, 2003, 2004, 2004, 2006, 2006, 2007, 2007, 2008, 2008, 2008, 2009, 2009, 2010, 2010, 2011, 2011, 2012,
	2012, 2012, 2014, 2014, 2015, 2015, 2016, 2016, 2017, 2017, 2017, 2018, 2018, 2019, 2019, 2021, 2021, 2021, 2022, 2022,
	2023, 2023, 2024, 2024, 2025, 2025, 2025, 2026, 2026, 2027, 2027, 2029, 2029, 2030, 2030, 2030, 2031, 2031, 2032, 2032,
	2033, 2033, 2033, 2034, 2034, 2035, 2035, 2037, 2037, 2038, 2038, 2038, 2039, 2039, 2040, 2040, 2041, 2041, 2041, 2042,
	2042, 2044, 2044, 2045, 2045, 2046, 2046, 2047
};

struct LCM_setting_table_by_type lcd_init_on[] = {
	{TYPE_DCS, 0x02, {0xFF, 0x23}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x00, 0x60}},
	{TYPE_DCS, 0x02, {0x07, 0x20}},
	{TYPE_DCS, 0x02, {0x08, 0x01}},
	{TYPE_DCS, 0x02, {0x09, 0x5A}},

	/* APL_THD */
	{TYPE_DCS, 0x02, {0x11, 0x02}},
	{TYPE_DCS, 0x02, {0x12, 0x87}},

	/* APL_COMP */
	{TYPE_DCS, 0x02, {0x15, 0x41}},
	{TYPE_DCS, 0x02, {0x16, 0x14}},
	{TYPE_DCS, 0x02, {0x0A, 0x8E}},
	{TYPE_DCS, 0x02, {0x0B, 0x8E}},
	{TYPE_DCS, 0x02, {0x0C, 0x8E}},
	{TYPE_DCS, 0x02, {0x0D, 0x00}},

	/* CABC_GMA */
	{TYPE_DCS, 0x02, {0x19, 0x00}},
	{TYPE_DCS, 0x02, {0x1A, 0x04}},
	{TYPE_DCS, 0x02, {0x1B, 0x08}},
	{TYPE_DCS, 0x02, {0x1C, 0x0C}},
	{TYPE_DCS, 0x02, {0x1D, 0x10}},
	{TYPE_DCS, 0x02, {0x1E, 0x14}},
	{TYPE_DCS, 0x02, {0x1F, 0x18}},
	{TYPE_DCS, 0x02, {0x20, 0x1C}},
	{TYPE_DCS, 0x02, {0x21, 0x20}},
	{TYPE_DCS, 0x02, {0x22, 0x24}},
	{TYPE_DCS, 0x02, {0x23, 0x28}},
	{TYPE_DCS, 0x02, {0x24, 0x2C}},
	{TYPE_DCS, 0x02, {0x25, 0x30}},
	{TYPE_DCS, 0x02, {0x26, 0x34}},
	{TYPE_DCS, 0x02, {0x27, 0x38}},
	{TYPE_DCS, 0x02, {0x28, 0x3C}},

	/* GAMMACMP */
	{TYPE_DCS, 0x02, {0x2A, 0x20}},
	{TYPE_DCS, 0x02, {0x2B, 0x30}},

	/* CABC_PWM_UI */
	{TYPE_DCS, 0x02, {0x30, 0xFF}},
	{TYPE_DCS, 0x02, {0x31, 0xFD}},
	{TYPE_DCS, 0x02, {0x32, 0xFC}},
	{TYPE_DCS, 0x02, {0x33, 0xFA}},
	{TYPE_DCS, 0x02, {0x34, 0xF8}},
	{TYPE_DCS, 0x02, {0x35, 0xF6}},
	{TYPE_DCS, 0x02, {0x36, 0xF4}},
	{TYPE_DCS, 0x02, {0x37, 0xF2}},
	{TYPE_DCS, 0x02, {0x38, 0xF0}},
	{TYPE_DCS, 0x02, {0x39, 0xEE}},
	{TYPE_DCS, 0x02, {0x3A, 0xEC}},
	{TYPE_DCS, 0x02, {0x3B, 0xEA}},
	{TYPE_DCS, 0x02, {0x3D, 0xE9}},
	{TYPE_DCS, 0x02, {0x3F, 0xE8}},
	{TYPE_DCS, 0x02, {0x40, 0xE7}},
	{TYPE_DCS, 0x02, {0x41, 0xE6}},

	/* CABC_PWM_STILL */
	{TYPE_DCS, 0x02, {0x45, 0xFF}},
	{TYPE_DCS, 0x02, {0x46, 0xF9}},
	{TYPE_DCS, 0x02, {0x47, 0xF6}},
	{TYPE_DCS, 0x02, {0x48, 0xF2}},
	{TYPE_DCS, 0x02, {0x49, 0xF0}},
	{TYPE_DCS, 0x02, {0x4A, 0xEC}},
	{TYPE_DCS, 0x02, {0x4B, 0xE8}},
	{TYPE_DCS, 0x02, {0x4C, 0xE4}},
	{TYPE_DCS, 0x02, {0x4D, 0xE0}},
	{TYPE_DCS, 0x02, {0x4E, 0xDE}},
	{TYPE_DCS, 0x02, {0x4F, 0xD9}},
	{TYPE_DCS, 0x02, {0x50, 0xD6}},
	{TYPE_DCS, 0x02, {0x51, 0xD4}},
	{TYPE_DCS, 0x02, {0x52, 0xD2}},
	{TYPE_DCS, 0x02, {0x53, 0xD0}},
	{TYPE_DCS, 0x02, {0x54, 0xCD}},

	/* CABC_PWM_MOV }*/
	{TYPE_DCS, 0x02, {0x58, 0xFF}},
	{TYPE_DCS, 0x02, {0x59, 0xF6}},
	{TYPE_DCS, 0x02, {0x5A, 0xF0}},
	{TYPE_DCS, 0x02, {0x5B, 0xEB}},
	{TYPE_DCS, 0x02, {0x5C, 0xE8}},
	{TYPE_DCS, 0x02, {0x5D, 0xE5}},
	{TYPE_DCS, 0x02, {0x5E, 0xE3}},
	{TYPE_DCS, 0x02, {0x5F, 0xE0}},
	{TYPE_DCS, 0x02, {0x60, 0xDE}},
	{TYPE_DCS, 0x02, {0x61, 0xDA}},
	{TYPE_DCS, 0x02, {0x62, 0xD7}},
	{TYPE_DCS, 0x02, {0x63, 0xD4}},
	{TYPE_DCS, 0x02, {0x64, 0xD2}},
	{TYPE_DCS, 0x02, {0x65, 0xD0}},
	{TYPE_DCS, 0x02, {0x66, 0xCC}},
	{TYPE_DCS, 0x02, {0x67, 0xC8}},

	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x07, {0x3B, 0x03, 0x14, 0x36, 0x0A, 0x0A, 0x00}},
	{TYPE_DCS, 0x02, {0x90, 0x03}},
	{TYPE_DCS, 0x11, {0x91, 0x89, 0xA8, 0x00, 0x08, 0xD2, 0x00, 0x00, 0x00, 0x00, 0xD8, 0x00, 0x0B, 0x0E, 0xDC, 0x07, 0xEB}},
	{TYPE_DCS, 0x03, {0x92, 0x10, 0xE0}},
	{TYPE_DCS, 0x02, {0x9D, 0x01}},
	{TYPE_DCS, 0x02, {0xB2, 0x00}},
	{TYPE_DCS, 0x03, {0x51, 0x07, 0xFF}},
	{TYPE_DCS, 0x02, {0x53, 0x2C}},
	{TYPE_DCS, 0x02, {0x55, 0x01}},

	/*esd config*/
	{TYPE_DCS, 0x02, {0xFF, 0x26}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x0A, {0x2D, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0F, 0x04, 0x00}},

	{TYPE_DCS, 0x02, {0xFF, 0x27}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xD0, 0x11}},
	{TYPE_DCS, 0x02, {0xD1, 0x54}},
	{TYPE_DCS, 0x02, {0xDE, 0x42}},
	{TYPE_DCS, 0x02, {0xDF, 0x00}},
	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x35, 0x00}},

	/*TP config*/
	{TYPE_DCS, 0x02, {0xFF, 0x24}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x04, {0x96, 0x88, 0xAA, 0x80}},
	{TYPE_DCS, 0x02, {0xFF, 0x26}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x07, {0x3D, 0x00, 0x00, 0xA0, 0x00, 0x00, 0x20}},
	{TYPE_DCS, 0x02, {0xFF, 0x27}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x03, {0x77, 0xAA, 0xA0}},
	{TYPE_DCS, 0x03, {0x97, 0xAA, 0xA0}},

	/*RGB MUX*/
	{TYPE_DCS, 0x02, {0xFF, 0x25}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x3B, 0x10}},
	{TYPE_DCS, 0x02, {0xFF, 0x27}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0xE2, 0xFF}},

	{TYPE_DCS, 0x02, {0xFF, 0x2A}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x30, 0x03}},
	{TYPE_DCS, 0x02, {0x33, 0xFE}},
	{TYPE_DCS, 0x02, {0x35, 0x0C}},

	{TYPE_DCS, 0x02, {0xFF, 0x24}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x5B, 0x71}},
	{TYPE_DCS, 0x04, {0x92, 0x7D,0x00,0xF5}},
	{TYPE_DCS, 0x03, {0x93, 0x1E,0x00}},
	{TYPE_DCS, 0x03, {0x94, 0x0A,0x00}},
	{TYPE_DCS, 0x04, {0xAA, 0xA3,0xA3,0x28}},

	{TYPE_DCS, 0x02, {0xFF, 0x25}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x14, 0x8D}},
	{TYPE_DCS, 0x02, {0x16, 0xD4}},
	{TYPE_DCS, 0x02, {0x20, 0x71}},
	{TYPE_DCS, 0x02, {0x23, 0x05}},
	{TYPE_DCS, 0x02, {0x24, 0x1D}},
	{TYPE_DCS, 0x02, {0x27, 0x71}},
	{TYPE_DCS, 0x02, {0x2A, 0x05}},
	{TYPE_DCS, 0x02, {0x2B, 0x1D}},
	{TYPE_DCS, 0x02, {0x34, 0x71}},
	{TYPE_DCS, 0x02, {0x37, 0x05}},
	{TYPE_DCS, 0x02, {0x38, 0x1D}},
	{TYPE_DCS, 0x02, {0x39, 0x08}},
	{TYPE_DCS, 0x02, {0x49, 0x71}},
	{TYPE_DCS, 0x02, {0x4C, 0x05}},
	{TYPE_DCS, 0x02, {0x4D, 0x1D}},
	{TYPE_DCS, 0x02, {0x4E, 0x08}},
	{TYPE_DCS, 0x02, {0x51, 0x71}},
	{TYPE_DCS, 0x02, {0x54, 0x05}},
	{TYPE_DCS, 0x02, {0x55, 0x1D}},
	{TYPE_DCS, 0x02, {0x56, 0x08}},
	{TYPE_DCS, 0x02, {0x62, 0x71}},
	{TYPE_DCS, 0x02, {0x65, 0x05}},
	{TYPE_DCS, 0x02, {0x66, 0x1D}},
	{TYPE_DCS, 0x02, {0x67, 0x08}},
	{TYPE_DCS, 0x02, {0x68, 0x04}},
	{TYPE_DCS, 0x02, {0xDC, 0xE9}},
	{TYPE_DCS, 0x02, {0xDE, 0x6F}},

	{TYPE_DCS, 0x02, {0xFF, 0x26}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x05, {0x19, 0x1D,0x1C,0x1C,0x1C}},
	{TYPE_DCS, 0x05, {0x1A, 0x12,0xED,0xED,0xED}},
	{TYPE_DCS, 0x05, {0x1B, 0x1C,0x1C,0x1C,0x1C}},
	{TYPE_DCS, 0x05, {0x1C, 0x32,0x0D,0x0D,0x0D}},
	{TYPE_DCS, 0x02, {0x1F, 0x7D}},
	{TYPE_DCS, 0x05, {0x2A, 0x1D,0x1C,0x1C,0x1C}},
	{TYPE_DCS, 0x05, {0x2B, 0x0A,0xE5,0xE5,0xE5}},
	{TYPE_DCS, 0x02, {0x30, 0x7D}},
	{TYPE_DCS, 0x02, {0x3A, 0x7D}},
	{TYPE_DCS, 0x02, {0x40, 0xCA}},
	{TYPE_DCS, 0x02, {0x42, 0xCA}},
	{TYPE_DCS, 0x02, {0x46, 0xCA}},
	{TYPE_DCS, 0x02, {0x48, 0xCA}},
	{TYPE_DCS, 0x02, {0x4A, 0xCA}},
	{TYPE_DCS, 0x02, {0x52, 0xBD}},
	{TYPE_DCS, 0x02, {0x58, 0xBD}},
	{TYPE_DCS, 0x02, {0x5C, 0xBD}},
	{TYPE_DCS, 0x02, {0x61, 0xBD}},
	{TYPE_DCS, 0x02, {0x65, 0xBD}},
	{TYPE_DCS, 0x02, {0x6A, 0xBD}},
	{TYPE_DCS, 0x02, {0x73, 0xBD}},
	{TYPE_DCS, 0x04, {0x84, 0x34,0x34,0x34}},
	{TYPE_DCS, 0x05, {0x99, 0x1D,0x1D,0x1D,0x1D}},
	{TYPE_DCS, 0x05, {0x9A, 0xE1,0xB8,0xB8,0xB8}},
	{TYPE_DCS, 0x05, {0x9B, 0x1D,0x1C,0x1C,0x1C}},
	{TYPE_DCS, 0x05, {0x9C, 0x01,0xD8,0xD8,0xD8}},
	{TYPE_DCS, 0x05, {0x9D, 0x1D,0x1D,0x1D,0x1D}},
	{TYPE_DCS, 0x05, {0x9E, 0xD9,0xB0,0xB0,0xB0}},

	{TYPE_DCS, 0x02, {0xFF, 0x27}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x5E, 0x17}},
	{TYPE_DCS, 0x03, {0x80, 0x62,0x0B}},
	{TYPE_DCS, 0x03, {0x83, 0x0B,0x00}},
	{TYPE_DCS, 0x03, {0x84, 0x27,0x00}},
	{TYPE_DCS, 0x03, {0x88, 0x27,0x00}},

	{TYPE_DCS, 0x02, {0xFF, 0x2A}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x14, 0x07}},
	{TYPE_DCS, 0x02, {0x15, 0x07}},
	{TYPE_DCS, 0x02, {0x16, 0x34}},
	{TYPE_DCS, 0x02, {0x17, 0x07}},
	{TYPE_DCS, 0x02, {0x18, 0x34}},
	{TYPE_DCS, 0x02, {0x19, 0x07}},
	{TYPE_DCS, 0x02, {0x1A, 0x34}},
	{TYPE_DCS, 0x02, {0x1B, 0x07}},
	{TYPE_DCS, 0x02, {0x1C, 0x34}},
	{TYPE_DCS, 0x02, {0x1D, 0x06}},
	{TYPE_DCS, 0x02, {0x1E, 0x07}},
	{TYPE_DCS, 0x02, {0x1F, 0x07}},
	{TYPE_DCS, 0x02, {0x30, 0x03}},
	{TYPE_DCS, 0x02, {0x32, 0xC9}},
	{TYPE_DCS, 0x02, {0x33, 0xFE}},
	{TYPE_DCS, 0x02, {0x35, 0x0C}},

	{TYPE_DCS, 0x02, {0xC5, 0x05}},
	{TYPE_DCS, 0x02, {0xC6, 0x1D}},
	{TYPE_DCS, 0x02, {0xC8, 0x06}},
	{TYPE_DCS, 0x02, {0xCA, 0x01}},
	{TYPE_DCS, 0x05, {0xCB, 0x11,0x00,0x00,0x00}},
	{TYPE_DCS, 0x07, {0xCC, 0xF5,0xAA,0x5F,0xF5,0xAA,0x5F}},
	{TYPE_DCS, 0x02, {0xD0, 0x04}},
	{TYPE_DCS, 0x02, {0xD1, 0x01}},
	{TYPE_DCS, 0x02, {0xDF, 0x01}},
	{TYPE_DCS, 0x02, {0xE2, 0x01}},
	{TYPE_DCS, 0x02, {0xF4, 0x91}},
	{TYPE_DCS, 0x02, {0xF5, 0x0A}},

	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x07, {0x3B, 0x03,0x0A,0x1E,0x0A,0x0A,0x00}},

	{TYPE_DCS, 0x02, {0xFF, 0x20}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x2D, 0x02}},
	{TYPE_DCS, 0x02, {0x2F, 0x02}},
	{TYPE_DCS, 0x02, {0x32, 0x72}},
	{TYPE_DCS, 0x02, {0x0D, 0x43}},

	{TYPE_DCS, 0x02, {0xFF, 0x2A}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x02, {0x9A, 0x05}},

	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x01, {0x11}},
	{TYPE_MDELAY, 120, {}},
	{TYPE_DCS, 0x01, {0x29}},
	{TYPE_MDELAY, 10, {}},
};

struct LCM_setting_table_by_type lcd_init_off[] = {
	{TYPE_DCS, 0x02, {0xFF, 0x10}},
	{TYPE_DCS, 0x02, {0xFB, 0x01}},
	{TYPE_DCS, 0x01, {0x28}},
	{TYPE_MDELAY, 60, {}},
	{TYPE_DCS, 0x01, {0x10}},
	{TYPE_MDELAY, 120, {}}
};

static void boe_panel_init_on(struct boe *ctx)
{
	int i= 0;

	printk("[lcd_info]%s +\n", __func__);

	for (i = 0; i < sizeof(lcd_init_on) / sizeof(struct LCM_setting_table_by_type); i++) {
		boe_init_write_by_type(ctx, &lcd_init_on[i]);
	}

	printk("[lcd_info]%s -\n", __func__);
}

static void boe_panel_init_off(struct boe *ctx)
{
	int i= 0;

	printk("[lcd_info]%s +\n", __func__);

	for (i = 0; i < sizeof(lcd_init_off) / sizeof(struct LCM_setting_table_by_type); i++) {
		boe_init_write_by_type(ctx, &lcd_init_off[i]);
	}

	printk("[lcd_info]%s -\n", __func__);
}

static int lcm_backlight_ic_config(struct device *pdev, int enable)
{

	struct device *dev = pdev;
	struct gpio_desc *hw_led_en;

	if (enable) {
		hw_led_en = devm_gpiod_get(dev, "pm-enable", GPIOD_OUT_HIGH);
		if (IS_ERR(hw_led_en))
			pr_debug("could not get pm-enable gpio\n");

		gpiod_set_value(hw_led_en, 1);
		devm_gpiod_put(dev, hw_led_en);
		usleep_range(125, 130);

		lcd_bl_write_byte(0x08, 0x00);
		//write vsp/vsn reg
		lcd_bl_write_byte(0x0C, 0x2A); /* LCD_BOOST_CFG */
		lcd_bl_write_byte(0x0D, 0x20); /* OUTP_CFG，OUTP = 5.6V */
		lcd_bl_write_byte(0x0E, 0x20); /* OUTN_CFG，OUTN = -5.6V */

		lcd_bl_write_byte(0x09, 0x9C); /* enable OUTP */
		mdelay(5); /* delay 5ms */
		lcd_bl_write_byte(0x09, 0x9E); /* enable OUTN */

		//write backlight reg
		/* BL_CFG1；OVP=29V，非线性调光，PWM Disabled */
		lcd_bl_write_byte(0x02, 0xB1);
		lcd_bl_write_byte(0x03, 0xCD);
		/* BL_OPTION2；电感4.7uH，BL_CURRENT_LIMIT 2.5A；*/
		lcd_bl_write_byte(0x11, 0x37);
		/* Backlight Full-scale LED Current 22.8mA/CH；*/
		lcd_bl_write_byte(0x15, 0xB0);
		lcd_bl_write_byte(0x04, 0x00);
		lcd_bl_write_byte(0x05, 0x00);

	} else {
		hw_led_en = devm_gpiod_get(dev, "pm-enable", GPIOD_OUT_HIGH);
		if (IS_ERR(hw_led_en))
			pr_debug("could not get pm-enable gpio\n");

		lcd_bl_write_byte(0x09, 0x9C);/* Disable OUTN */
		mdelay(5);
		lcd_bl_write_byte(0x09, 0x98);/* Disable OUTP */
		mdelay(10);
		/* BL disabled and Current sink 1/2/3/4 /5 enabled；*/
		lcd_bl_write_byte(0x08, 0x00);

		mdelay(20);
		gpiod_set_value(hw_led_en, 0);
		devm_gpiod_put(dev, hw_led_en);

	}

	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel, int en)
{
	struct boe *ctx = panel_to_boe(panel);

	printk("%s + en:%d\n", __func__, en);
	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_LOW);
	if (en) {
		if (esd_flag) {
			gpiod_set_value(ctx->reset_gpio,1);
			usleep_range(5 * 1000, 10 * 1000);
			gpiod_set_value(ctx->reset_gpio,0);
			usleep_range(10 * 1000, 15 * 1000);
			gpiod_set_value(ctx->reset_gpio,1);
			usleep_range(55 * 1000, 65 * 1000);
		} else {
			gpiod_set_value(ctx->reset_gpio,0);
			usleep_range(3 * 1000, 5 * 1000);
			gpiod_set_value(ctx->reset_gpio,1);
			usleep_range(5 * 1000, 10 * 1000);
			gpiod_set_value(ctx->reset_gpio,0);
			usleep_range(10 * 1000, 15 * 1000);
			gpiod_set_value(ctx->reset_gpio,1);
			usleep_range(55 * 1000, 65 * 1000);
		}
	} else {
		if (esd_flag || shut_down_flag) {
			printk("[lcd_info]%s:trigger esd and shutdown, lcm reset off\n", __func__);
			gpiod_set_value(ctx->reset_gpio,0);
		} else {
			printk("[lcd_info]%s:lcm reset not off\n", __func__);
		}
	}
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);
	printk("%s -\n", __func__);
	return 0;
}

static int boe_disable(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int boe_unprepare(struct drm_panel *panel)
{

	struct boe *ctx = panel_to_boe(panel);
	int blank;

	pr_info("%s+\n", __func__);
	if (!ctx->prepared)
		return 0;

	if ((mode == MSM_BOOT_MODE__FACTORY) || (mode == MSM_BOOT_MODE__RF) || (mode == MSM_BOOT_MODE__WLAN)) {
		blank = LCD_CTL_RST_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_err("[TP] Now mode is %d, reset will low\n", mode);
		blank = LCD_CTL_CS_OFF;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_err("[TP]TP CS will chang to gpio mode and low\n");
	} else {
		if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0)) {
			tp_gusture_mode = true;
			pr_err("[TP] tp gesture  is enable,Display not to poweroff\n");
		} else {
			tp_gusture_mode = false;
			blank = LCD_CTL_RST_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
			blank = LCD_CTL_CS_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_err("[TP]TP CS will chang to gpio mode and low\n");
		}
	}

	boe_panel_init_off(ctx);

	pr_err("lcm_suspend_power, shut_down_flag:%d\n", shut_down_flag);
	if (esd_flag || shut_down_flag) {
		pr_err("[LCM] trigger esd power off or doing shutdown\n");

		lcm_panel_reset(panel, 0);

		lcm_backlight_ic_config(ctx->dev, 0);

		ctx->bias_neg =
			devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_LOW);
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(2000, 2001);

		ctx->bias_pos =
			devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_LOW);
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);

		/* disable 1.8V */
		ctx->v1v8_enable_gpio = devm_gpiod_get(ctx->dev, "lcm1v8", GPIOD_OUT_LOW);
		gpiod_set_value(ctx->v1v8_enable_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->v1v8_enable_gpio);
	} else {
		lcd_bl_write_byte(0x08, 0x00);
	}

	ctx->error = 0;
	ctx->prepared = false;
	pr_info("%s+\n", __func__);

	return 0;
}

static int boe_prepare(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);
	int ret;
	int blank;

	pr_info("%s+\n", __func__);
	if (ctx->prepared)
		return 0;

	if (esd_flag) {
		pr_info("[ESD] in dis_panel_power_on,esd_flag = %d\n", esd_flag);
		lcm_backlight_ic_config(ctx->dev, 1);

		ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);

		usleep_range(2000, 2001);
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 1);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);
		usleep_range( 10* 1000, 15 * 1000);
	}

	lcm_panel_reset(panel, 1);

	pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
	if ((mode != MSM_BOOT_MODE__FACTORY) && (mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
		blank = LCD_CTL_CS_ON;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_err("[TP]TP CS will chang to spi mode and high\n");
		usleep_range(5000, 5100);
		blank = LCD_CTL_TP_LOAD_FW;
		mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
		pr_info("[TP] start to load fw!\n");
	}

	boe_panel_init_on(ctx);

	/* BL enabled and Current sink 1/2/3/4 /5 enabled；*/
	lcd_bl_write_byte(0x08, 0x7F);
	if (esd_flag) {
		lcd_bl_set_led_brightness(bl_last_level);
	}

	ret = ctx->error;
	if (ret < 0)
		boe_unprepare(panel);

	ctx->prepared = true;
#ifdef PANEL_SUPPORT_READBACK
	boe_panel_get_data(ctx);
#endif



	pr_info("%s-\n", __func__);
	return ret;
}

static int boe_enable(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP (74)
#define HSA (28)
#define HBP (76)
#define VSA (2)
#define VBP (8)
#define VAC (2408)
#define HAC (1720)
#define VFP_48hz (2172)
#define VFP_50hz (1988)
#define VFP_60hz (1254)
#define VFP_90hz (30)
#define DYN_DATA_RATE (1080)
#define DYN_HBP (56)
#define DATA_RATE (1107)

#define HFP_TABB (496)
#define VFP_60hz_TABB (30)
#define VFP_50hz_TABB (518)
#define VFP_48hz_TABB (642)


static const struct drm_display_mode default_mode = {
	.clock = 416845,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,//HFP
	.hsync_end = HAC + HFP + HSA,//HSA
	.htotal = HAC + HFP + HSA + HBP,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_90hz,//VFP
	.vsync_end = VAC + VFP_90hz + VSA,//VSA
	.vtotal = VAC + VFP_90hz + VSA + VBP,//VBP
};

static const struct drm_display_mode performance_mode_60hz = {
	.clock = 418167,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,//HFP
	.hsync_end = HAC + HFP + HSA,//HSA
	.htotal = HAC + HFP + HSA + HBP,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_60hz,//VFP
	.vsync_end = VAC + VFP_60hz + VSA,//VSA
	.vtotal = VAC + VFP_60hz + VSA + VBP,//VBP
};

static const struct drm_display_mode performance_mode_50hz = {
	.clock = 418129,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,//HFP
	.hsync_end = HAC + HFP + HSA,//HSA
	.htotal = HAC + HFP + HSA + HBP,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_50hz,//VFP
	.vsync_end = VAC + VFP_50hz + VSA,//VSA
	.vtotal = VAC + VFP_50hz + VSA + VBP,//VBP
};

static const struct drm_display_mode performance_mode_48hz = {
	.clock = 418167,
	.hdisplay = HAC,
	.hsync_start = HAC + HFP,//HFP
	.hsync_end = HAC + HFP + HSA,//HSA
	.htotal = HAC + HFP + HSA + HBP,//HBP
	.vdisplay = VAC,
	.vsync_start = VAC + VFP_48hz,//VFP
	.vsync_end = VAC + VFP_48hz + VSA,//VSA
	.vtotal = VAC + VFP_48hz + VSA + VBP,//VBP
};

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params_60hz = {
	.lcm_index = 0,
//	.pll_clk = 580,
//	.vfp_low_power = 1298,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	.ssc_enable = 0,
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2408,
		.pic_width = 1720,
		.slice_height = 8,
		.slice_width = 860,
		.chunk_size = 860,
		.xmit_delay = 512,
		.dec_delay = 723,
		.scale_value = 32,
		.increment_interval = 216,
		.decrement_interval = 11,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 3804,
		.slice_bpg_offset = 2027,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 10,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 11,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 11,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 9,
		.rc_range_parameters[13].range_max_qp = 12,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 12,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
	},
	.data_rate = 1107,
	.rotate = MTK_PANEL_ROTATE_180,
	.lfr_enable = 0,
	.lfr_minimum_fps = 60,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		//.dfps_cmd_table[0] = {0, 4, {0xB9, 0x83, 0x10, 0x21} },
	//	.dfps_cmd_table[1] = {0, 2, {0xE2, 0x00} },
	//	.dfps_cmd_table[2] = {0, 2, {0xB9, 0x00} },
		/*switch page for esd check*/
	//	.dfps_cmd_table[3] = {0, 2, {0xFF, 0x10} },
	//	.dfps_cmd_table[4] = {0, 2, {0xFB, 0x01} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vfp = VFP_60hz,
		.vbp = VBP,
		.hsa = HSA,
		.hfp = HFP,
		.hbp = DYN_HBP,
	},
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.vendor = "23971_NT36532",
        .manufacture = "23971_boe",
};

static struct mtk_panel_params ext_params_50hz = {
	.lcm_index = 0,
//	.pll_clk = 580,
//	.vfp_low_power = 2044,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	.ssc_enable = 0,
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2408,
		.pic_width = 1720,
		.slice_height = 8,
		.slice_width = 860,
		.chunk_size = 860,
		.xmit_delay = 512,
		.dec_delay = 723,
		.scale_value = 32,
		.increment_interval = 216,
		.decrement_interval = 11,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 3804,
		.slice_bpg_offset = 2027,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 10,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 11,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 11,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 9,
		.rc_range_parameters[13].range_max_qp = 12,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 12,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
	},
	.data_rate = 1107,
	.rotate = MTK_PANEL_ROTATE_180,
	.lfr_enable = 0,
	.lfr_minimum_fps = 50,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 50,
		//.dfps_cmd_table[0] = {0, 4, {0xB9, 0x83, 0x10, 0x21} },
		//.dfps_cmd_table[1] = {0, 2, {0xE2, 0x20} },
		//.dfps_cmd_table[2] = {0, 2, {0xB9, 0x00} },
		/*switch page for esd check*/
		//.dfps_cmd_table[3] = {0, 2, {0xFF, 0x10} },
		//.dfps_cmd_table[4] = {0, 2, {0xFB, 0x01} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vfp = VFP_50hz,
		.vbp = VBP,
		.hsa = HSA,
		.hfp = HFP,
		.hbp = DYN_HBP,
	},
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.vendor = "23971_NT36532",
        .manufacture = "23971_boe",
};

static struct mtk_panel_params ext_params_48hz = {
	.lcm_index = 0,
	//.pll_clk = 580,
//	.vfp_low_power = 2231,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	.ssc_enable = 0,
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2408,
		.pic_width = 1720,
		.slice_height = 8,
		.slice_width = 860,
		.chunk_size = 860,
		.xmit_delay = 512,
		.dec_delay = 723,
		.scale_value = 32,
		.increment_interval = 216,
		.decrement_interval = 11,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 3804,
		.slice_bpg_offset = 2027,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 10,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 11,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 11,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 9,
		.rc_range_parameters[13].range_max_qp = 12,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 12,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
	},
	.data_rate = 1107,
	.rotate = MTK_PANEL_ROTATE_180,
	.lfr_enable = 0,
	.lfr_minimum_fps = 48,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 48,
		//.dfps_cmd_table[0] = {0, 4, {0xB9, 0x83, 0x10, 0x21} },
		//.dfps_cmd_table[1] = {0, 2, {0xE2, 0x20} },
		//.dfps_cmd_table[2] = {0, 2, {0xB9, 0x00} },
		/*switch page for esd check*/
		//.dfps_cmd_table[3] = {0, 2, {0xFF, 0x10} },
		//.dfps_cmd_table[4] = {0, 2, {0xFB, 0x01} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vfp = VFP_48hz,
		.vbp = VBP,
		.hsa = HSA,
		.hfp = HFP,
		.hbp = DYN_HBP,
	},
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.vendor = "23971_NT36532",
        .manufacture = "23971_boe",
};

static struct mtk_panel_params ext_params = {
	.lcm_index = 0,
//	.pll_clk = 580,
	.vfp_low_power = VFP_60hz,
	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.esd_te_check_gpio = 1,
	//.lcm_esd_check_table[0] = {
	//	.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C,
	//},
	.ssc_enable = 0,
	.lane_swap_en = 0,
	.lane_swap[0][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[0][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[0][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[0][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[0][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[0][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_0] = MIPITX_PHY_LANE_0,
	.lane_swap[1][MIPITX_PHY_LANE_1] = MIPITX_PHY_LANE_1,
	.lane_swap[1][MIPITX_PHY_LANE_2] = MIPITX_PHY_LANE_3,
	.lane_swap[1][MIPITX_PHY_LANE_3] = MIPITX_PHY_LANE_2,
	.lane_swap[1][MIPITX_PHY_LANE_CK] = MIPITX_PHY_LANE_CK,
	.lane_swap[1][MIPITX_PHY_LANE_RX] = MIPITX_PHY_LANE_0,
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,
	.dsc_params = {
		.enable = 1,
		.ver = 18,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 34,
		.rct_on = 1,
		.bit_per_channel = 8,
		.dsc_line_buf_depth = 9,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2408,
		.pic_width = 1720,
		.slice_height = 8,
		.slice_width = 860,
		.chunk_size = 860,
		.xmit_delay = 512,
		.dec_delay = 723,
		.scale_value = 32,
		.increment_interval = 216,
		.decrement_interval = 11,
		.line_bpg_offset = 13,
		.nfl_bpg_offset = 3804,
		.slice_bpg_offset = 2027,
		.initial_offset = 6144,
		.final_offset = 4320,
		.flatness_minqp = 3,
		.flatness_maxqp = 12,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 11,
		.rc_quant_incr_limit1 = 11,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.rc_buf_thresh[0] = 14,
		.rc_buf_thresh[1] = 28,
		.rc_buf_thresh[2] = 42,
		.rc_buf_thresh[3] = 56,
		.rc_buf_thresh[4] = 70,
		.rc_buf_thresh[5] = 84,
		.rc_buf_thresh[6] = 98,
		.rc_buf_thresh[7] = 105,
		.rc_buf_thresh[8] = 112,
		.rc_buf_thresh[9] = 119,
		.rc_buf_thresh[10] = 121,
		.rc_buf_thresh[11] = 123,
		.rc_buf_thresh[12] = 125,
		.rc_buf_thresh[13] = 126,
		.rc_range_parameters[0].range_min_qp = 0,
		.rc_range_parameters[0].range_max_qp = 4,
		.rc_range_parameters[0].range_bpg_offset = 2,
		.rc_range_parameters[1].range_min_qp = 0,
		.rc_range_parameters[1].range_max_qp = 4,
		.rc_range_parameters[1].range_bpg_offset = 0,
		.rc_range_parameters[2].range_min_qp = 1,
		.rc_range_parameters[2].range_max_qp = 5,
		.rc_range_parameters[2].range_bpg_offset = 0,
		.rc_range_parameters[3].range_min_qp = 1,
		.rc_range_parameters[3].range_max_qp = 6,
		.rc_range_parameters[3].range_bpg_offset = -2,
		.rc_range_parameters[4].range_min_qp = 3,
		.rc_range_parameters[4].range_max_qp = 7,
		.rc_range_parameters[4].range_bpg_offset = -4,
		.rc_range_parameters[5].range_min_qp = 3,
		.rc_range_parameters[5].range_max_qp = 7,
		.rc_range_parameters[5].range_bpg_offset = -6,
		.rc_range_parameters[6].range_min_qp = 3,
		.rc_range_parameters[6].range_max_qp = 7,
		.rc_range_parameters[6].range_bpg_offset = -8,
		.rc_range_parameters[7].range_min_qp = 3,
		.rc_range_parameters[7].range_max_qp = 8,
		.rc_range_parameters[7].range_bpg_offset = -8,
		.rc_range_parameters[8].range_min_qp = 3,
		.rc_range_parameters[8].range_max_qp = 9,
		.rc_range_parameters[8].range_bpg_offset = -8,
		.rc_range_parameters[9].range_min_qp = 3,
		.rc_range_parameters[9].range_max_qp = 10,
		.rc_range_parameters[9].range_bpg_offset = -10,
		.rc_range_parameters[10].range_min_qp = 5,
		.rc_range_parameters[10].range_max_qp = 10,
		.rc_range_parameters[10].range_bpg_offset = -10,
		.rc_range_parameters[11].range_min_qp = 5,
		.rc_range_parameters[11].range_max_qp = 11,
		.rc_range_parameters[11].range_bpg_offset = -12,
		.rc_range_parameters[12].range_min_qp = 5,
		.rc_range_parameters[12].range_max_qp = 11,
		.rc_range_parameters[12].range_bpg_offset = -12,
		.rc_range_parameters[13].range_min_qp = 9,
		.rc_range_parameters[13].range_max_qp = 12,
		.rc_range_parameters[13].range_bpg_offset = -12,
		.rc_range_parameters[14].range_min_qp = 12,
		.rc_range_parameters[14].range_max_qp = 13,
		.rc_range_parameters[14].range_bpg_offset = -12
	},
	.data_rate = 1107,
	.rotate = MTK_PANEL_ROTATE_180,
	.lfr_enable = 0,
	.lfr_minimum_fps = 90,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		//.dfps_cmd_table[0] = {0, 4, {0xB9, 0x83, 0x10, 0x21} },
		//.dfps_cmd_table[1] = {0, 2, {0xE2, 0x10} },
		//.dfps_cmd_table[2] = {0, 2, {0xB9, 0x00} },
		/*switch page for esd check*/
	//	.dfps_cmd_table[3] = {0, 2, {0xFF, 0x10} },
	//	.dfps_cmd_table[4] = {0, 2, {0xFB, 0x01} },
	},
	/* following MIPI hopping parameter might cause screen mess */
	.dyn = {
		.switch_en = 1,
		.data_rate = DYN_DATA_RATE,
		.vsa = VSA,
		.vfp = VFP_90hz,
		.vbp = VBP,
		.hsa = HSA,
		.hfp = HFP,
		.hbp = DYN_HBP,
	},
	.corner_pattern_height = ROUND_CORNER_H_TOP,
	.corner_pattern_tp_size = sizeof(top_rc_pattern),
	.corner_pattern_lt_addr = (void *)top_rc_pattern,
	.vendor = "23971_NT36532",
        .manufacture = "23971_boe",
};
static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct boe *ctx = panel_to_boe(panel);

	printk("[lcd_info][23971]%s +\n", __func__);

	if (ctx->prepared){
		pr_err("[lcd_info]%s frist time ctx->prepared=%d\n", __func__, ctx->prepared);
		return 0;
	}

	if (esd_flag) {
		usleep_range(30 * 1000, 40 * 1000);
	}
	//set vddi 1.8v
	ctx->v1v8_enable_gpio = devm_gpiod_get(ctx->dev, "lcm1v8", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->v1v8_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->v1v8_enable_gpio);
	usleep_range(2 * 1000, 10 * 1000);

	pr_info("[lcd_info][23971]%s -\n", __func__);
	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static int boe_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	static bool flag = true;

	if(level > 2047)
		level = 2047;

	if (level) {
		if (flag) {
			flag = false;
			mdelay(20);
			pr_info("boe_setbacklight_cmdq first set bl delay 20ms level = %d", level);
		}
	} else {
		flag = true;
	}

	pr_info("%s- set brightness level = %d --> map_level = %d\n", __func__,level,backlight_mapping_buf[level]);

	level = backlight_mapping_buf[level];
	lcd_bl_set_led_brightness(level);
	bl_last_level = level;

	return 0;
}

struct drm_display_mode *get_mode_by_id_hfp(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}
static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id_hfp(connector, mode);
 	printk("wlc------> drm_mode_vrefresh(m) =%d",drm_mode_vrefresh(m));
	if (drm_mode_vrefresh(m) == 90)
		ext->params = &ext_params;
	else if (drm_mode_vrefresh(m) == 60)
		ext->params = &ext_params_60hz;
	else if (drm_mode_vrefresh(m) == 50)
		ext->params = &ext_params_50hz;
	else if (drm_mode_vrefresh(m) == 48)
		ext->params = &ext_params_48hz;
	else
		ret = 1;

	return ret;
}

#define CABC_MODE_CMD_SIZE 4
static void boe_cabc_mode_switch(void *dsi, dcs_write_gce cb,
		void *handle, unsigned int cabc_mode)
{
	unsigned char cabc_mode_para = 0;
	int i = 0;
	unsigned char cabc_mode_cmd[CABC_MODE_CMD_SIZE][2] = {
		{0xFF, 0x10},
		{0xFB, 0x01},
		{0x53, 0x2C},
		{0x55, 0x00},
	};

	switch(cabc_mode) {
		case 0:
			cabc_mode_para = 0;
			break;
		case 1:
			cabc_mode_para = 1;
			break;
		case 2:
			cabc_mode_para = 2;
			break;
		case 3:
			cabc_mode_para = 3;
			break;
		default:
			cabc_mode_para = 0;
			printk("[lcd_info]%s: cabc_mode=%d is not support, close cabc !\n", __func__, cabc_mode);
			break;
	}

	cabc_mode_cmd[3][1] = cabc_mode_para;
	for (i = 0; i < CABC_MODE_CMD_SIZE; i++) {
		cb(dsi, handle, cabc_mode_cmd[i], ARRAY_SIZE(cabc_mode_cmd[i]));
	}
	printk("[lcd_info]%s:cabc mode_%d, set cabc_para=%d\n", __func__, cabc_mode, cabc_mode_para);
}

static int lcd_esd_gpio_read(struct drm_panel *panel)
{
	struct boe *ctx = container_of(panel, struct boe, panel);
	int ret = 0;

	if(gpiod_get_value(ctx->esd_te_gpio)) {
		pr_err("[ESD]%s: triger esd to recovery\n", __func__);
		esd_flag = true;
		ret = 1;
	} else {
		esd_flag = false;
		ret = 0;
	}

	if (1 == ret) {
		char payload[100] = "";
		int cnt = 0;

		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "DisplayDriverID@@507$$");
		cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, "ESD failed");
		pr_err("ESD check failed: %s\n", payload);
		mm_fb_display_kevent(payload, MM_FB_KEY_RATELIMIT_1H, "ESD check failed");
	}

	pr_err("[ESD]%s:ret=%d\n", __func__, ret);
	return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
	struct boe *ctx = panel_to_boe(panel);

	ctx->reset_gpio =
		devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	gpiod_set_value(ctx->reset_gpio, on);
	devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = boe_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.panel_poweron = lcm_panel_poweron,
	.cabc_switch = boe_cabc_mode_switch,
	.ata_check = panel_ata_check,
	.esd_read_gpio = lcd_esd_gpio_read,
};
#endif

struct panel_desc {
	const struct drm_display_mode *modes;
	unsigned int num_modes;

	unsigned int bpc;

	struct {
		unsigned int width;
		unsigned int height;
	} size;

	/**
	 * @prepare: the time (in milliseconds) that it takes for the panel to
	 *	   become ready and start receiving video data
	 * @enable: the time (in milliseconds) that it takes for the panel to
	 *	  display the first valid frame after starting to receive
	 *	  video data
	 * @disable: the time (in milliseconds) that it takes for the panel to
	 *	   turn the display off (no content is visible)
	 * @unprepare: the time (in milliseconds) that it takes for the panel
	 *		 to power itself down completely
	 */
	struct {
		unsigned int prepare;
		unsigned int enable;
		unsigned int disable;
		unsigned int unprepare;
	} delay;
};

static int boe_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	struct drm_display_mode *mode2;
	struct drm_display_mode *mode3;
	struct drm_display_mode *mode4;

	pr_info("%s+ \n", __func__);
	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 default_mode.hdisplay, default_mode.vdisplay,
			 drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	mode2 = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
	if (!mode2) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_60hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode2);
	mode2->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode2);

	mode3 = drm_mode_duplicate(connector->dev, &performance_mode_50hz);
	if (!mode3) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_50hz.hdisplay, performance_mode_50hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_50hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode3);
	mode3->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode3);

	mode4 = drm_mode_duplicate(connector->dev, &performance_mode_48hz);
	if (!mode4) {
		dev_info(connector->dev->dev, "failed to add mode %ux%ux@%u\n",
			 performance_mode_48hz.hdisplay, performance_mode_48hz.vdisplay,
			 drm_mode_vrefresh(&performance_mode_48hz));
		return -ENOMEM;
	}

	drm_mode_set_name(mode4);
	mode4->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode4);

	connector->display_info.width_mm = 168;
	connector->display_info.height_mm = 235;

	pr_info("%s-- \n", __func__);
	return 1;
}

static const struct drm_panel_funcs boe_drm_funcs = {
	.disable = boe_disable,
	.unprepare = boe_unprepare,
	.prepare = boe_prepare,
	.enable = boe_enable,
	.get_modes = boe_get_modes,
};

static int boe_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct boe *ctx;
	struct device_node *backlight;
	int ret;

	pr_info("%s+ boe,nt36532,vdo,90hz\n", __func__);

	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct boe), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_EOT_PACKET;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	ctx->v1v8_enable_gpio = devm_gpiod_get(ctx->dev, "lcm1v8", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->v1v8_enable_gpio)) {
		printk("[lcd_info]cannot get reset-gpios %ld\n",
			 PTR_ERR(ctx->v1v8_enable_gpio));
		return PTR_ERR(ctx->v1v8_enable_gpio);
	}

	/* enable bias_enn_en +6V */
	ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_pos)) {
		dev_info(dev, "cannot get bias-gpios 0 %ld\n",
			PTR_ERR(ctx->bias_pos));
		return PTR_ERR(ctx->bias_pos);
	}
	devm_gpiod_put(dev, ctx->bias_pos);

	/* enable bias_enn_en -6V */
	ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->bias_neg)) {
		dev_info(dev, "cannot get bias-gpios 1 %ld\n",
			PTR_ERR(ctx->bias_neg));
		return PTR_ERR(ctx->bias_neg);
	}
	devm_gpiod_put(dev, ctx->bias_neg);

	gpiod_set_value(ctx->v1v8_enable_gpio, 1);
	devm_gpiod_put(ctx->dev, ctx->v1v8_enable_gpio);
	msleep(10);

	ctx->esd_te_gpio = devm_gpiod_get(ctx->dev, "esd-te", GPIOD_IN);
	if (IS_ERR(ctx->esd_te_gpio)) {
		dev_info(dev,"[lcd_info]cannot get esd_te_gpio %ld\n",
			PTR_ERR(ctx->esd_te_gpio));
		return PTR_ERR(ctx->esd_te_gpio);
	}
	gpiod_direction_input(ctx->esd_te_gpio);

	ctx->prepared = true;
	ctx->enabled = true;
	drm_panel_init(&ctx->panel, dev, &boe_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;

#endif

	mode = get_boot_mode();
       //add proc/devinfo/lcd
        register_device_proc("lcd", "23971_NT36532", "23971_boe");

	pr_info("%s- boe,nt36532,vdo,90hz\n", __func__);

	return ret;
}

static int boe_remove(struct mipi_dsi_device *dsi)
{
	struct boe *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

	return 0;
}

static const struct of_device_id boe_of_match[] = {
	{
		.compatible = "boe,nt36532,vdo,90hz",
	},
	{}
};

MODULE_DEVICE_TABLE(of, boe_of_match);

static struct mipi_dsi_driver boe_driver = {
	.probe = boe_probe,
	.remove = boe_remove,
	.driver = {
		.name = "oplus23971_boe_nt36532_vdo_90hz",
		.owner = THIS_MODULE,
		.of_match_table = boe_of_match,
	},
};

module_mipi_dsi_driver(boe_driver);

MODULE_AUTHOR("wulongchao <wulongchao@huaqin.com>");
MODULE_DESCRIPTION("BOE NT36532 VDO 90HZ LCD Panel Driver");
MODULE_LICENSE("GPL v2");

