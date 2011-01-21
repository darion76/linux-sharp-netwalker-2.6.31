#ifndef __MX51_EFIKAMX_H__
#define __MX51_EFIKAMX_H__

#define VIDEO_OUT_STATIC_AUTO	0
#define VIDEO_OUT_STATIC_HDMI	1
#define VIDEO_OUT_STATIC_DSUB	2

#define res_matches_refresh(v, x, y, r) \
			((v).xres == (x) && (v).yres == (y) && (v).refresh == (r))


#endif /* __MX51_EFIKAMX_H__ */