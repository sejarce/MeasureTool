import QtQuick.Window 2.0 
import QtQuick 2.7 
import QtQuick.Dialogs 1.2
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0

import Cloudream.Controls 1.0

Rectangle {
	
	property var windowType: -1
	
	function setText(str, type)
	{
		text_msg.text = str
		windowType = type
	}
	
	Rectangle{
		width: 26
		height: 24
		anchors.left: parent.left
		anchors.leftMargin: 21
		anchors.top: parent.top
		anchors.topMargin: 39
		color: "#2f3033"
		Image{
			anchors.fill:parent;
			source: "res/icon_caution_n.png"
		}
	}
	
	Text{
		id: text_msg
		text: ""	
		width: 292
		height: 42
		anchors.top: parent.top
		anchors.topMargin: 30
		anchors.left: parent.left
		anchors.leftMargin: 63
		font.pixelSize: 15
		color:"#FFFFFF";
		font.family:"微软雅黑"
	}
	
	Row{
		anchors.left: parent.left
		anchors.leftMargin: 44
		anchors.top: parent.top
		anchors.topMargin: 108
		spacing: 23
		Button{
			id: b_save
			width: 82
			height: 32
			Rectangle{
				anchors.fill: parent
				color: "#444547"
				Text{
					text: "保存"	
					width: parent.width
					height: 21
					anchors.top: parent.top
					anchors.topMargin: 5
					anchors.left: parent.left
					anchors.leftMargin: 26
					font.pixelSize: 15
					color:"#FFFFFF";
					font.family:"微软雅黑"
				}
			}				
			onClicked:{
				saveWin.hide()
				saveWin.visible = false
				if(windowType == 0)
				{				
					dataStatusChanged(1)
				}else if(windowType == 1)
				{
					g_next_openorclose = 0
					root.saveFile()					
					//if (EventWrapper.openFileDlg())
					//{
					//	root.disableToolBar()
					//}
				}else if(windowType == 2)
				{
					g_next_openorclose = 1
					root.saveFile()					
					//mainWindow.close()
				}
			}
		}
		Button{
			id: b_nosave
			width: 82
			height: 32
			Rectangle{
				anchors.fill: parent
				color: "#444547"
				Text{
					text: "不保存"	
					width: parent.width
					height: 21
					anchors.top: parent.top
					anchors.topMargin: 5
					anchors.left: parent.left
					anchors.leftMargin: 20
					font.pixelSize: 15
					color:"#FFFFFF";
					font.family:"微软雅黑"
				}
			}				
			onClicked:{	
				saveWin.hide()
				saveWin.visible = false
				if(windowType == 0)
				{
					dataStatusChanged(-1)
				}else if(windowType == 1)
				{
					if (EventWrapper.openFileDlg())
					{
						root.disableToolBar()
					}
				}else if(windowType == 2)
				{
					mainWindow.close()
				}
			}
		}
		Button{
			id: b_cancel
			width: 82
			height: 32
			Rectangle{
				anchors.fill: parent
				color: "#444547"
				Text{
					text: "取消"	
					width: parent.width
					height: 21
					anchors.top: parent.top
					anchors.topMargin: 5
					anchors.left: parent.left
					anchors.leftMargin: 26
					font.pixelSize: 15
					color:"#FFFFFF";
					font.family:"微软雅黑"
				}
			}				
			onClicked:{
				saveWin.hide()
				saveWin.visible = false
				dataStatusChanged(0)
			}
		}			
	}
}