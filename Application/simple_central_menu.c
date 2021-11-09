#include <bcomdef.h>
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "simple_central_menu.h"
#include "simple_central.h"
#include "ti_ble_config.h"

/* Main Menu Object */
tbmMenuObj_t scMenuMain;
tbmMenuObj_t scMenuConnect;
tbmMenuObj_t scMenuScanPhy;
tbmMenuObj_t scMenuAutoConnect;
tbmMenuObj_t scMenuConnPhy;
tbmMenuObj_t scMenuSelectConn;
tbmMenuObj_t scMenuPerConn;
tbmMenuObj_t scMenuGattWrite;

/*
 * Menu Lists Initializations
 */
void SimpleCentral_buildMenu(void)
{
// Menu: Main
// upper: none
  MENU_OBJ(scMenuMain, "ESLO Speaker", 6, NULL)
//  MENU_ITEM_SUBMENU(scMenuMain,0,&scMenuScanPhy)
  MENU_ITEM_ACTION(scMenuMain,0,"Discover Devices",  SimpleCentral_doDiscoverDevices)
  MENU_ITEM_ACTION(scMenuMain,1,"Stop Discovering",  SimpleCentral_doStopDiscovering)
  MENU_ITEM_SUBMENU(scMenuMain,2,&scMenuConnect)
  MENU_ITEM_ACTION(scMenuMain,3,"Cancel Connecting", SimpleCentral_doCancelConnecting)
  MENU_ITEM_SUBMENU(scMenuMain,4,&scMenuSelectConn)
  MENU_ITEM_SUBMENU(scMenuMain,5,&scMenuAutoConnect)
  MENU_OBJ_END

// Menu: ScanPhy
// upper: Main
  MENU_OBJ(scMenuScanPhy, "Set Scanning PHY", 2, &scMenuMain)
  MENU_ITEM_ACTION(scMenuScanPhy,0,"1 Mbps", SimpleCentral_doSetScanPhy)
  MENU_ITEM_ACTION(scMenuScanPhy,1,"Coded",  SimpleCentral_doSetScanPhy)
  MENU_OBJ_END
    
// Menu: AutoConnect
// upper: Main
  MENU_OBJ(scMenuAutoConnect, "Set AutoConnect", 2, &scMenuMain)
  MENU_ITEM_ACTION(scMenuAutoConnect,0,"Disable", SimpleCentral_doAutoConnect)
  MENU_ITEM_ACTION(scMenuAutoConnect,1,"Group ES", SimpleCentral_doAutoConnect)
  MENU_OBJ_END

// Menu: Connect
// upper: Main
// NOTE: The number of items in this menu object shall be equal to
//       or greater than DEFAULT_MAX_SCAN_RES.
//       The number of items cannot exceed 27 which is the two-button menu's
//       constraint.
  MENU_OBJ(scMenuConnect, "Connect to", DEFAULT_MAX_SCAN_RES, &scMenuMain)
  MENU_ITEM_MULTIPLE_ACTIONS(scMenuConnect, DEFAULT_MAX_SCAN_RES, NULL, SimpleCentral_doConnect)
  MENU_OBJ_END

// Menu: SelectDev
// upper: Main
// NOTE: The number of items in this menu object shall be
//       equal to or greater than MAX_NUM_BLE_CONNS
  MENU_OBJ(scMenuSelectConn, "Work with", MAX_NUM_BLE_CONNS, &scMenuMain)
  MENU_ITEM_MULTIPLE_ACTIONS(scMenuSelectConn, MAX_NUM_BLE_CONNS, NULL, SimpleCentral_doSelectConn)
  MENU_OBJ_END

// Menu: PerConnection
// upper: SelectDevice
  MENU_OBJ(scMenuPerConn, NULL, 2, &scMenuSelectConn)
//  MENU_ITEM_ACTION(scMenuPerConn,0,"GATT Read",          SimpleCentral_doGattRead)
//  MENU_ITEM_SUBMENU(scMenuPerConn,1,&scMenuGattWrite)
  MENU_ITEM_ACTION(scMenuPerConn,0,"Enable Indications", SimpleCentral_enableIndications)
//  MENU_ITEM_ACTION(scMenuPerConn,3,"Start RSSI Reading", SimpleCentral_doRssiRead)
//  MENU_ITEM_ACTION(scMenuPerConn,4,"Stop RSSI Reading",  SimpleCentral_doRssiRead)
//  MENU_ITEM_ACTION(scMenuPerConn,5,"Connection Update",  SimpleCentral_doConnUpdate)
//  MENU_ITEM_SUBMENU(scMenuPerConn,6,&scMenuConnPhy)
  MENU_ITEM_ACTION(scMenuPerConn,1,"Disconnect",         SimpleCentral_doDisconnect)
  MENU_OBJ_END

// Menu: GattWrite
// upper: PerConnection
  MENU_OBJ(scMenuGattWrite, "GATT Write", 2, &scMenuPerConn)
  MENU_ITEM_ACTION(scMenuGattWrite,0,"Write 0x00", SimpleCentral_doGattWrite)
  MENU_ITEM_ACTION(scMenuGattWrite,1,"Write 0x01", SimpleCentral_doGattWrite)
  MENU_OBJ_END

// Menu: ConnPhy
// upper: Main
  MENU_OBJ(scMenuConnPhy, "Set Conn PHY Preference", 5, &scMenuMain)
  MENU_ITEM_ACTION(scMenuConnPhy,0,"1 Mbps",              SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,1,"2 Mbps",              SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,2,"1 & 2 Mbps",          SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,3,"Coded",               SimpleCentral_doSetConnPhy)
  MENU_ITEM_ACTION(scMenuConnPhy,4,"1 & 2 Mbps, & Coded", SimpleCentral_doSetConnPhy)
  MENU_OBJ_END
}
