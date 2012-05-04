#ifndef LPC31XX_DT
#define LPC31XX_DT

void lpc31xx_dt_init_common(struct of_dev_auxdata* auxdata);
void lpc31xx_init_early(void);
void lpc31xx_restart(char mode, const char *cmd);

#endif

