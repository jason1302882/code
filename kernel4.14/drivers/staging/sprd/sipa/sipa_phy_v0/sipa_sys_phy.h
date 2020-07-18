#ifndef _IPA_SYS_PHY_H_
#define _IPA_SYS_PHY_H_

#include "../sipa_hal_priv.h"

#define IPA_SYS_AHB_REG_BASE_ADDR	0x21040000l

/*ipa sys ahb soft reset*/
#define IPA_PCIE2_SOFT_RST_MASK		0x00000040l
#define IPA_PCIE3_SOFT_RST_MASK		0x00000020l
#define IPA_IPA_SOFT_RST_MASK		0x00000010l
#define IPA_PAM_WIFI_SOFT_RST_MASK	0x00000008l
#define IPA_PAM_IPA_SOFT_RST_MASK	0x00000004l
#define IPA_PAM_U3_SOFT_RST_MASK	0x00000002l
#define IPA_USB_SOFT_RST_MASK		0x00000001l
#define IPA_SOFT_RST_GROUP_MASK		0x0000007Fl
#define IPA_SYS_AHB_RST			(0x00l)

/*ipa sys shb eb*/
#define IPA_PCIE2_EB_MASK		0x00000100l
#define IPA_PCIE3_EB_MASK		0x00000080l
#define IPA_PAM_WIFI_EB_MASK		0x00000040l
#define IPA_PAM_IPA_EB_MASK		0x00000020l
#define IPA_PAM_USB_EB_MASK		0x00000010l
#define IPA_IPA_EB_MASK			0x00000008l
#define IPA_USB_REF_EB_MASK		0x00000004l
#define IPA_USB_SUSPEND_EB_MASK		0x00000002l
#define IPA_USB_EB_MASK			0x00000001l
#define IPA_SYS_AHB_ENABLE		(0x04l)

#define IPA_MSI2IPA_SET_MASK		0x00000080l
#define IPA_G3_APP_RAS_DES_SD_HOLD_ITSSM_MASK \
					0x00000040l

#define IPA_G3_APP_RAS_DES_TBA_CTRL_MASK \
					0x00000030l

#define IPA_G3_APP_L1_PWR_OFF_EN_MASK \
					0x00000008l

#define IPA_G3_APP_XFER_PENDING_MASK \
					0x00000004l

#define IPA_G3_DBG_TABLE_MASK		0x00000002l
#define IPA_G3_DBG_PBA_MASK		0x00000001l
#define IPA_SYS_PCIE3_CTL0		(0x08l)

#define IPA_G2_APP_RAS_DES_SD_HOLD_ITSSM_MASK \
					0x00000040l

#define IPA_G2_APP_RAS_DES_TBA_CTRL_MASK \
					0x00000030l

#define IPA_G2_APP_L1_PWR_OFF_EN_MASK \
					0x00000008l

#define IPA_G2_APP_XFER_PENDING_MASK \
					0x00000004l

#define IPA_G2_DBG_TABLE_MASK		0x00000002l
#define IPA_G2_DBG_PBA_MASK		0x00000001l
#define IPA_SYS_PCIE2_CTL0		(0x0Cl)

#define IPA_XHC_BME_MASK		0x00001000l
#define IPA_BUS_FILTER_BYPASS_MASK	0x00000F00l
#define IPA_PME_EN_MASK			0x00000080l
#define IPA_USB_FLADJ_30MHZ_REG_MASK	0x0000007El
#define IPA_USB_HOST_PORT_POWER_COMTROL_PRESENT_MASK \
					0x00000001l

#define IPA_SYS_USB_CTL0		(0x10l)

#define IPA_MAIN_M0_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M0_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M0_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M0_LPC			(0x14l)

#define IPA_MAIN_M1_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M1_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M1_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M1_LPC			(0x18l)

#define IPA_MAIN_M2_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M2_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M2_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M2_LPC			(0x1Cl)

#define IPA_MAIN_M3_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M3_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M3_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M3_LPC			(0x20l)

#define IPA_MAIN_M4_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M4_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M4_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M4_LPC			(0x24l)

#define IPA_MAIN_M5_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M5_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M5_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M5_LPC			(0x28l)

#define IPA_MAIN_M6_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M6_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M6_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M6_LPC			(0x2Cl)

#define IPA_MAIN_M7_PU_NUM_MASK		0xFF000000l
#define IPA_MAIN_M7_LP_EB_MASK		0x00010000l
#define IPA_MAIN_M7_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_M7_LPC			(0x30l)

#define IPA_MAIN_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MATRIX_AUTO_GATE_EN_MASK \
					0x00020000l

#define IPA_MAIN_LP_EB_MASK		0x00010000l
#define IPA_MAIN_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_MAIN_MTX_LPC		(0x34l)

#define IPA_MAIN_S0_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S0_AUTO_GATE_EN_MASK \
					0x00020000l
#define IPA_MAIN_S0_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S0_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S0_LPC			(0x38l)

#define IPA_MAIN_S1_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S1_AUTO_GATE_EN_MASK \
					0x00020000l

#define IPA_MAIN_S1_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S1_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S1_LPC			(0x3Cl)

#define IPA_MAIN_S2_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S2_AUTO_GATE_EN_MASK \
					0x00020000l

#define IPA_MAIN_S2_LP_EB_MASK		0x00010000l
#define IPA_SYS_S2_LPC			(0x40l)

#define IPA_MAIN_S3_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S3_AUTO_GATE_EN_MASK 0x00020000l
#define IPA_MAIN_S3_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S3_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S3_LPC			(0x44l)

#define IPA_MAIN_S4_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S4_AUTO_GATE_EN_MASK 0x00020000l
#define IPA_MAIN_S4_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S4_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S4_LPC			(0x48l)

#define IPA_MAIN_S5_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S5_AUTO_GATE_EN_MASK 0x00020000l
#define IPA_MAIN_S5_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S5_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S5_LPC			(0x4Cl)

#define IPA_MAIN_S6_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S6_AUTO_GATE_EN_MASK 0x00020000l
#define IPA_MAIN_S6_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S6_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S6_LPC			(0x50l)

#define IPA_MAIN_S7_PU_NUM_MASK		0xFF000000l
#define IPA_CGM_MTX_S7_AUTO_GATE_EN_MASK 0x00020000l
#define IPA_MAIN_S7_LP_EB_MASK		0x00010000l
#define IPA_MAIN_S7_LP_NUM_MASK		0x0000FFFFl
#define IPA_SYS_S7_LPC			(0x54l)

#define IPA_ASYNC_BRG_G2_PU_NUM_MASK	0xFF000000l
#define IPA_ASYNC_BRG_G2_LP_EB_MASK	0x00010000l
#define IPA_ASYNC_BRG_G2_LP_NUM_MASK	0x0000FFFFl
#define IPA_SYS_ASYNC_BRG_G2_IPC	(0x58l)

#define IPA_ASYNC_BRG_G3_PU_NUM_MASK	0xFF000000l
#define IPA_ASYNC_BRG_G3_LP_EB_MASK	0x00010000l
#define IPA_ASYNC_BRG_G3_LP_NUM_MASK	0x0000FFFFl
#define IPA_SYS_ASYNC_BRG_G3_IPC	(0x5Cl)

#define IPA_AWQOS_M4_MASK		0xF0000000l
#define IPA_ARQOS_M4_MASK		0x0F000000l
#define IPA_AWQOS_M3_MASK		0x00F00000l
#define IPA_ARQOS_M3_MASK		0x000F0000l
#define IPA_AWQOS_M2_MASK		0x0000F000l
#define IPA_ARQOS_M2_MASK		0x00000F00l
#define IPA_AWQOS_M1_MASK		0x000000F0l
#define IPA_ARQOS_M1_MASK		0x0000000Fl
#define IPA_SYS_ASYNC_BRG_QOS_0		(0x60l)

#define IPA_AWQOS_M7_MASK		0x00F00000l
#define IPA_ARQOS_M7_MASK		0x000F0000l
#define IPA_AWQOS_M6_MASK		0x0000F000l
#define IPA_ARQOS_M6_MASK		0x00000F00l
#define IPA_AWQOS_M5_MASK		0x000000F0l
#define IPA_ARQOS_M5_MASK		0x0000000Fl
#define IPA_SYS_ASYNC_BRG_QOS_1		(0x64l)

#define IPA_APB_FRC_OFF_MASK		0x00000200l
#define IPA_APB_FRC_ON_MASK		0x00000100l
#define IPA_MTX_FRC_OFF_MASK		0x00000080l
#define IPA_MTX_FRC_ON_MASK		0x00000040l
#define IPA_MTX_FREQ_LSLP_DFS_SEL_MASK	0x00000030l
#define IPA_MTX_FREQ_LSLP_DFS_EN_MASK	0x00000008l
#define IPA_MTX_FREQ_LSLP_DFS_MODE_MASK	0x00000004l
#define IPA_MTX_FREQ_DFS_MODE_MASK	0x00000002l
#define IPA_AXI_LP_CTRL_DISABLE_MASK	0x00000001l
#define IPA_SYS_MATRIX_LPC_CTRL		(0x68l)

#define IPA_IMTX_M7_AXI_FREQ_ALLOW_MASK	0xF0000000l
#define IPA_IMTX_M6_AXI_FREQ_ALLOW_MASK	0x0F000000l
#define IPA_IMTX_M5_AXI_FREQ_ALLOW_MASK	0x00F00000l
#define IPA_IMTX_M4_AXI_FREQ_ALLOW_MASK	0x000F0000l
#define IPA_IMTX_M3_AXI_FREQ_ALLOW_MASK	0x0000F000l
#define IPA_IMTX_M2_AXI_FREQ_ALLOW_MASK	0x00000F00l
#define IPA_IMTX_M1_AXI_FREQ_ALLOW_MASK	0x000000F0l
#define IPA_IMTX_M0_AXI_FREQ_ALLOW_MASK	0x0000000Fl
#define IPA_SYS_MTX_FREQ_ALLOW		(0x6Cl)

#define IPA_IMTX_M7_AXI_FREQ_LSLP_ALLOW_MASK 0xF0000000l
#define IPA_IMTX_M6_AXI_FREQ_LSLP_ALLOW_MASK 0x0F000000l
#define IPA_IMTX_M5_AXI_FREQ_LSLP_ALLOW_MASK 0x00F00000l
#define IPA_IMTX_M4_AXI_FREQ_LSLP_ALLOW_MASK 0x000F0000l
#define IPA_IMTX_M3_AXI_FREQ_LSLP_ALLOW_MASK 0x0000F000l
#define IPA_IMTX_M2_AXI_FREQ_LSLP_ALLOW_MASK 0x00000F00l
#define IPA_IMTX_M1_AXI_FREQ_LSLP_ALLOW_MASK 0x000000F0l
#define IPA_IMTX_M0_AXI_FREQ_LSLP_ALLOW_MASK 0x0000000Fl
#define IPA_SYS_MTX_FREQ_LSLP_ALLOW	 (0x70l)

#define IPA_AWQOS_THRESHOLD_IPA2_MASK	0x0000F000l
#define IPA_ARQOS_THRESHOLD_IPA2_MASK	0x00000F00l
#define IPA_AWQOS_THRESHOLD_IPA1_MASK	0x000000F0l
#define IPA_ARQOS_THRESHOLD_IPA1_MASK	0x0000000Fl
#define IPA_SYS_ASYNC_BRG_QOS_THRESHOLD	(0x74l)

#define IPA_GEN3_BRIDGE_TRANS_IDLE_MASK		0x00000008l
#define IPA_GEN3_AXI_DETECTOR_OVERFLOW_MASK	0x00000004l
#define IPA_GEN2_BRIDGE_TRANS_IDLE_MASK		0x00000002l
#define IPA_GEN2_AXI_DETECTOR_OVERFLOW_MASK	0x00000001l
#define IPA_SYS_STATUS				(0x78l)

#define IPA_SYS_AP2IPA_BRIDGE_DEBUG_SIGNAL_R	(0x7Cl)
#define IPA_SYS_GEN2_BRIDGE_DEBUG_SIGNAL_W	(0x80l)
#define IPA_SYS_GEN3_BRIDGE_DEBUG_SIGNAL_W	(0x84l)
#define IPA_SYS_USB3_DEBUG_0			(0x88l)
#define IPA_SYS_USB3_DEBUG_1			(0x8Cl)

#define IPA_USB3_CLK_GATE_CTRL_MASK		0x00000070l
#define IPA_USB3_HOST_SYSTEM_ERR_MASK		0x00000008l
#define IPA_USB3_DEBUG_64_66_MASK		0x00000007l
#define IPA_SYS_USB3_DEBUG_2			(0x90l)

#define IPA_G2_CFG_INT_DISABLE_MASK		0x01000000l
#define IPA_G2_CFG_PCIE_CAP_INT_MSG_NUM_MASK	0x00F80000l
#define IPA_G2_CFG_AER_INT_MSG_NUM_MASK		0x0007C000l
#define IPA_G2_CFG_PWR_IND_MASK			0x00003000l
#define IPA_G2_CFG_ATTEN_IND_MASK		0x00000C00l
#define IPA_G2_CFG_PWR_CTRLER_CTRL_MASK		0x00000200l
#define IPA_G2_CFG_BUS_MASTER_EN_MASK		0x00000100l
#define IPA_G2_PCIE_DBG_SIGNAL_MASK		0x000000FFl
#define IPA_SYS_PCIE2_DEBUG			(0x94l)

#define IPA_G3_CFG_INT_DISABLE_MASK		0x01000000l
#define IPA_G3_CFG_PCIE_CAP_INT_MSG_NUM_MASK	0x00F80000l
#define IPA_G3_CFG_AER_INT_MSG_NUM_MASK		0x0007C000l
#define IPA_G3_CFG_PWR_IND_MASK			0x00003000l
#define IPA_G3_CFG_ATTEN_IND_MASK		0x00000C00l
#define IPA_G3_CFG_PWR_CTRLER_CTRL_MASK		0x00000200l
#define IPA_G3_CFG_BUS_MASTER_EN_MASK		0x00000100l
#define IPA_G3_PCIE_DBG_SIGNAL_MASK		0x000000FFl
#define IPA_SYS_PCIE3_DEBUG			(0x98l)

static inline u32 sipa_sys_phy_module_soft_rst(void __iomem *reg_base, u32 sys)
{
	u32 tmp;
	void __iomem *reg_addr;

	reg_addr = reg_base + IPA_SYS_AHB_RST;

	tmp = readl_relaxed(reg_addr);
	tmp |= sys;
	writel_relaxed(tmp, reg_addr);

	tmp = readl_relaxed(reg_addr);
	tmp &= (~sys);
	writel_relaxed(tmp, reg_addr);

	return TRUE;
}

static inline u32 sipa_sys_phy_module_enable(void __iomem *reg_base, u32 sys)
{
	u32 tmp;

	tmp = readl_relaxed(reg_base + IPA_SYS_AHB_ENABLE);
	tmp |= sys;
	writel_relaxed(tmp, reg_base + IPA_SYS_AHB_ENABLE);

	return TRUE;
}

static inline u32 sipa_sys_phy_module_disable(void __iomem *reg_base, u32 sys)
{
	u32 tmp;

	tmp = readl_relaxed(reg_base + IPA_SYS_AHB_ENABLE);
	tmp &= (~sys);
	writel_relaxed(tmp, reg_base + IPA_SYS_AHB_ENABLE);

	return TRUE;
}

#endif