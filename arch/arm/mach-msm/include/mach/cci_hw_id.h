#ifndef __CCI_HW_ID_H
#define __CCI_HW_ID_H

/* Implement CCI HW ID/PROJECT ID */

#define CCI_HWID_TYPE_STRLEN 20

#define GPIO_CCI_WATERPROOF_ID 34

#define HWID_DEVTREE_RPOPERTY_NAME_NUM		5
#define HWID_DEVTREE_RPOPERTY_NAME_SIZE		25
#define HWID_DEVTREE_GPIO_LABEL_NAME_SIZE		30

#define HWID_DEVTREE_PROPERTY_PROJECT_ID 		"cci,project_id-gpios"
#define HWID_DEVTREE_PROPERTY_HW_ID				"cci,hw_id-gpios"
#define HWID_DEVTREE_PROPERTY_SIM_ID			"cci,sim_id-gpios"
#define HWID_DEVTREE_PROPERTY_BAND_ID			"cci,band_id-gpios"
#define HWID_DEVTREE_PROPERTY_CUSTOMER_ID		"cci,customer_id-gpios"
#define HWID_DEVTREE_GPIO_LABEL_PROJECT_ID		"cci,project_id-gpio_labels"
#define HWID_DEVTREE_GPIO_LABEL_HW_ID			"cci,hw_id-gpio_labels"
#define HWID_DEVTREE_GPIO_LABEL_SIM_ID			"cci,sim_id-gpio_labels"
#define HWID_DEVTREE_GPIO_LABEL_BAND_ID			"cci,band_id-gpio_labels"
#define HWID_DEVTREE_GPIO_LABEL_CUSTOMER_ID	"cci,customer_id-gpio_labels"

enum cci_project_id_type {
	CCI_PROJECTID_VY52_53 = 0,
	CCI_PROJECTID_VY55_56 = 1,
	CCI_PROJECTID_VY58_59 = 2,
	CCI_PROJECTID_INVALID
};

enum cci_band_id_type {
	CCI_BANDID_EU = 0,
	CCI_BANDID_US = 1,
	CCI_BANDID_INVALID
};

enum cci_sim_id_type {
	CCI_SIMID_SS = 0, 		/* Single SIM */
	CCI_SIMID_DS = 1, 		/* Dual SIM */
	CCI_SIMID_INVALID
};

enum cci_hw_id_type {
	CCI_HWID_EVT 	 = 0,
	CCI_HWID_DVT1	 = 1,
	CCI_HWID_DVT1_1 = 2,
	CCI_HWID_DVT2 	 = 3,
	CCI_HWID_DVT3 	 = 4,
	CCI_HWID_PVT 	 = 5,
	CCI_HWID_MP 	 = 6,
	CCI_HWID_INVALID
};

enum cci_customer_id_type {
	CCI_CUSTOMERID_CUSTOMER = 0,
	CCI_CUSTOMERID_CCI = 1,
	CCI_CUSTOMERID_INVALID
};

extern int get_cci_hw_id(void); 
extern int get_cci_project_id(void);
extern int get_cci_customer_id(void);
extern int get_cci_band_id(void);
extern int get_cci_sim_id(void);
int cci_hw_sim_type_read( char *page, char **start, off_t off, int count, int *eof, void *data );
int cci_hw_board_type_read( char *page, char **start, off_t off, int count, int *eof, void *data );
int cci_hw_band_type_read( char *page, char **start, off_t off, int count, int *eof, void *data );
int cci_hw_proj_name_read( char *page, char **start, off_t off, int count, int *eof, void *data );
int cci_hw_customer_type_read( char *page, char **start, off_t off, int count, int *eof, void *data );
int cci_hwid_info_read( char *page, char **start, off_t off, int count, int *eof, void *data );

#endif /* __CCI_HW_ID_H */
