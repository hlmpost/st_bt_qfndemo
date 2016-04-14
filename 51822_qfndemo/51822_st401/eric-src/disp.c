//œ‘ æ¥¶¿Ì
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "EPD_TCON.h"
#include "Graphics.h"
#include "disp.h"

#define SCREEN_WIDTH    128 //1,43"
#define SCREEN_HEIGHT   296 //1,43"
#define ARRAY1DOT43   (9472)
#define ARRAYSIZE     ARRAY1DOT43

#define EDGE1_INDEX 8  //0~7   //#8  //(SCREEN_WIDTH/(16))
#define EDGE2_INDEX 24 //8~23  //#16 //(SCREEN_HEIGHT/16)-2
#define EDGE3_INDEX 32 //24~31 //#8
#define EDGE4_INDEX 48 //32~47 //#16


extern const EPD_TCON_DRIVER_HAL g_TCON_HAL;
extern const unsigned char g_WaveformData[];

static const EPD_TCON_PANEL_INFO *_paneInfo;
//--------------------------------------------------------
void DrawProgress(int32_t index, uint8_t color)
{
//#define SCREEN_WIDTH    128 //1,43"
//#define SCREEN_HEIGHT   296 //1,43"
#define RECT_W (12)
#define RECT_H (12)

    index %= EDGE4_INDEX;
    if (index < EDGE1_INDEX)
    {
        GR_FillRectangle(index * 16 + 2, 2, RECT_W, RECT_H, color);
        GR_DrawRectangle(index * 16 + 2, 2, RECT_W, RECT_H, 0x0);                
    }
    else if (index < EDGE2_INDEX)
    {
        index = index - (EDGE1_INDEX-1);
        GR_FillRectangle((SCREEN_WIDTH-14), index * 16 + 2, RECT_W, RECT_H, color);
        GR_DrawRectangle((SCREEN_WIDTH-14), index * 16 + 2, RECT_W, RECT_H, 0x0);                                
    }
    else if (index < EDGE3_INDEX)
    {
        index = index - EDGE2_INDEX;
        index = (EDGE1_INDEX-1) - index;
        GR_FillRectangle(index * 16 + 2, (SCREEN_HEIGHT-22), RECT_W, RECT_H, color);
        GR_DrawRectangle(index * 16 + 2, (SCREEN_HEIGHT-22), RECT_W, RECT_H, 0x0);                
    }
    else
    {
        index = index - EDGE3_INDEX;
        index = (EDGE2_INDEX-EDGE1_INDEX) - index;
        GR_FillRectangle(2, index * 16 + 2, RECT_W, RECT_H, color);
        GR_DrawRectangle(2, index * 16 + 2, RECT_W, RECT_H, 0x0);                                
    }    
}

//-------------------------------------------------------
void marquee()
{

    uint16_t digit[3];
    uint32_t i;     
    digit[2] = 0;
            

    EPD_TCON_PowerOn(); 
    GR_FillRectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0x03);
    
    for (i = 0; i < SCREEN_WIDTH; i += 8)
    {
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), i, 0, 0);
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), i, (SCREEN_HEIGHT - 1), 0);
    }
    for (i = 0; i < SCREEN_HEIGHT; i += 8)
    {
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), (SCREEN_WIDTH - 1), i, 0);
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), 0, i, 0);
    }
  
    for (i = 0; i < EDGE4_INDEX; i++)
    {
        DrawProgress(i + 0, 0x3);
        DrawProgress(i + 1, 0x0);   
    }
    EPD_TCON_Update(WF_MODE_GC4);    
                    
    for (i = 0; i < EDGE4_INDEX; i++)
    {
        digit[0] = '0' + ((i / 10) % 10);
        digit[1] = '0' + (i % 10);

        DrawProgress(i + 0, 0x3);
        DrawProgress(i + 1, 0x0);
        
                   
        EPD_TCON_Update(WF_MODE_A2);                  
         
    }
      
    EPD_TCON_PowerOff();
}
//---------------------------------------------------------
//disp test
void lcd_test()
{
    uint16_t digit[3];
    uint32_t temp=0;     
    digit[2] = 0;
            

    EPD_TCON_PowerOn(); 
    GR_FillRectangle(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, 0x00);
	  GR_FillRectangle(10, 10, SCREEN_WIDTH-20, SCREEN_HEIGHT-20, 0x03);
		EPD_TCON_Update(WF_MODE_A2); 
	
	
	for(int i=10;i>1;i--)
	{
		
		GR_FillRectangle(i+40, i+40, SCREEN_WIDTH-2*(i+40), SCREEN_HEIGHT-2*(i+40), temp);
		EPD_TCON_Update(WF_MODE_GC4); 
		if(temp==0)
			temp=3;
	}

    /*
    for (i = 0; i < SCREEN_WIDTH; i += 8)
    {
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), i, 0, 0);
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), i, (SCREEN_HEIGHT - 1), 0);
    }
    for (i = 0; i < SCREEN_HEIGHT; i += 8)
    {
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), (SCREEN_WIDTH - 1), i, 0);
        GR_DrawLine((SCREEN_WIDTH / 2), (SCREEN_HEIGHT / 2), 0, i, 0);
    }
  
    for (i = 0; i < EDGE4_INDEX; i++)
    {
        DrawProgress(i + 0, 0x3);
        DrawProgress(i + 1, 0x0);   
    }
		*/
    //EPD_TCON_Update(WF_MODE_GC4);    
    EPD_TCON_PowerOff();
}
//------------------------------------------------
//lcd init
uint8_t lcd_init()
{
		int32_t ret;   
    uint8_t *frameBuffer;
	

		//init lcd
	 //EPD_HAL_Init();
    ret = EPD_TCON_Init(&g_TCON_HAL);
    if ( ret < 0)
    {
        //TCON Init Fail!!
        while (1);
    }
		EPD_TCON_LoadWaveform(g_WaveformData);

    EPD_TCON_SetVcom(-2960);

    _paneInfo = EPD_TCON_GetPanelInfo();
		
		frameBuffer = EPD_TCON_GetUpdateBuffer();
    GR_Initial(frameBuffer);

    EPD_TCON_PowerOn();
    EPD_TCON_PanelClear();
    EPD_TCON_PowerOff();

}

