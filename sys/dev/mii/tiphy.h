#define	DP83867_PHYCR			0x10	/* PHY Control Register */
#define	 PHYCR_SGMII_EN			(1 << 11)
#define	DP83867_CFG2			0x14	/* Configuration Register 2 */
#define	 CFG2_SPEED_OPT_10M_EN		(1 << 6) /* Speed Optimization */
#define	 CFG2_SPEED_OPT_ENHANCED_EN	(1 << 8)
#define	 CFG2_SPEED_OPT_ATTEMPT_CNT_S	10
#define	 CFG2_SPEED_OPT_ATTEMPT_CNT_M	(0x3 << CFG2_SPEED_OPT_ATTEMPT_CNT_S)
#define	 CFG2_SPEED_OPT_ATTEMPT_CNT_1	(0 << CFG2_SPEED_OPT_ATTEMPT_CNT_S)
#define	 CFG2_SPEED_OPT_ATTEMPT_CNT_2	(1 << CFG2_SPEED_OPT_ATTEMPT_CNT_S)
#define	 CFG2_SPEED_OPT_ATTEMPT_CNT_4	(2 << CFG2_SPEED_OPT_ATTEMPT_CNT_S)
#define	 CFG2_SPEED_OPT_ATTEMPT_CNT_8	(3 << CFG2_SPEED_OPT_ATTEMPT_CNT_S)
#define	 CFG2_INTERRUPT_POLARITY	(1 << 13) /* Int pin is active low. */
#define	DP83867_CFG4			0x31 /* Configuration Register 4 */
