#ifndef CODE_MY_FLASH_H_
#define CODE_MY_FLASH_H_

#include "zf_common_headfile.h"

#define FLASH_SECTION_INDEX       (0)       // �洢�����õ�����
#define FLASH_PAGE_INDEX          (11)      // �����0-11���洢�����õ�ҳ�� ������һ��ҳ��

void read_flash_to_buffer(void);
void write_buffer_to_flash(void);

extern uint8 write_flag,flash_flag;
extern float speed;
#endif /* CODE_MY_FLASH_H_ */
