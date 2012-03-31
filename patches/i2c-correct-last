Bottom: aa28be9cf5344669994716a5f84b8a0f7e4c8455
Top:    b5627fb0e95bee04f2c99967cf8bfb9e97c06432
Author: Jon Smirl <jonsmirl@gmail.com>
Date:   2012-03-30 20:17:22 -0400




---

diff --git a/drivers/i2c/busses/i2c-pnx.c b/drivers/i2c/busses/i2c-pnx.c
index 83ae3b3..e4519d9 100644
--- a/drivers/i2c/busses/i2c-pnx.c
+++ b/drivers/i2c/busses/i2c-pnx.c
@@ -221,7 +221,8 @@ static int i2c_pnx_master_xmit(struct i2c_pnx_algo_data *alg_data)
 		/* We still have something to talk about... */
 		val = *alg_data->mif.buf++;
 
-		if (alg_data->mif.len == 1)
+		/* last byte of a message */
+		if ((alg_data->mif.len == 1) && alg_data->last)
 			val |= stop_bit;
 
 		alg_data->mif.len--;
