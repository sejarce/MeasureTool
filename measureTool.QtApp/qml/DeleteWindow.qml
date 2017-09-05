import QtQuick.Window 2.0 
import QtQuick 2.7 
import QtQuick.Dialogs 1.2
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0

import Cloudream.Controls 1.0

Rectangle {
	
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
		text: "是否删除当前项数据？"	
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
		spacing: 128
		Button{
			id: b_save
			width: 82
			height: 32
			Rectangle{
				anchors.fill: parent
				color: "#444547"
				Text{
					text: "确定"	
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
				//console.log("确定删除")
				deleteWin.hide()
				deleteWin.visible = false
				//dataStatusChanged(1)
				removeCurrData(true)
				
				root.setCheckToolBar(1)
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
				//console.log("取消删除")
				deleteWin.hide()
				deleteWin.visible = false
				//dataStatusChanged(0)
			}
		}			
	}
}