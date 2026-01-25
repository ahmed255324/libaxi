#ifndef AXI_H
#define AXI_H

#include <stdint.h>

#define ERROR_OK						(0)
#define ERROR_NO_CONFIG_FILE			(-2)
#define ERROR_BUF_TOO_SMALL				(-3)
/* see "Error:" log entry for meaningful message to the user. The caller should
 * make no assumptions about what went wrong and try to handle the problem.
 */
#define ERROR_FAIL						(-4)
#define ERROR_WAIT						(-5)
/* ERROR_TIMEOUT is already taken by winerror.h. */
#define ERROR_TIMEOUT_REACHED			(-6)
#define ERROR_NOT_IMPLEMENTED			(-7)


/* Low level flags */
#define COPY_TDO_BUFFER		(1 << 0)

struct axi_driver {
	char *device_desc;

	int (*write)(struct axi_driver *low, uint8_t *buf, int size,
		     uint32_t *bytes_written);
	int (*read)(struct axi_driver *low, uint8_t *buf, unsigned size,
		    uint32_t *bytes_read);
	int (*open)(struct axi_driver *low);
	int (*close)(struct axi_driver *low);
	int (*speed)(struct axi_driver *low, int speed);

	int flags;
};


extern struct axi_driver *axi_driver_register(void);

#endif /* AXI_H */
