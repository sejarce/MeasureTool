import QtQuick 2.0 
import Cloudream.Controls 1.0

Rectangle {
	id: rect_item
	width: 134
	height: 32
	color: "#2f3033"
	
	property var textName: ""
	property var textData: ""
		
	function clickOn()
	{
		rect_item.color = "black"	//"#2A2B2E"
	}
	
	function reset()
	{
		rect_item.color = "#2f3033"
	}
	
	function updateValue(val)
	{
		text_item_val.text = val + "cm"
	}
	
	function updateName(name)
	{
		text_item.text = name
	}
	
	signal clickSignal();					
	onClickSignal: {
		//console.log("item index " + index)
		//console.log("list Model count " + listModel_data.count);
		for (var i = 0; i < listModel_data.count; i++) {
			var item = listView_data.itemAt(rect_item.width / 2, 16 + 32 * i)
			if (i == index) {
				item.clickOn();
			} else {
				item.reset();
			}
		}
		//清空下面新增数据选中状态
		for (var i = 0; i < listModel.count; i++) {
			var item = listView.itemAt(rect_item.width / 2, 16 + 32 * i)
			item.reset();
		}
	}
	
	Rectangle{
		id: icon_item 
		width: 4
		height: 4
		anchors.left: parent.left
		anchors.leftMargin: 10
		anchors.top: parent.top
		anchors.topMargin: 14
		color: "#FFFFFF"
	}
	Text{
		id: text_item
		text: textName
		width: 55
		anchors.top: parent.top
		anchors.topMargin: 8
		anchors.left: parent.left
		anchors.leftMargin: 20
		font.pixelSize: 14
		font.letterSpacing: -0.04
		color:"#FFFFFF";
		opacity: 0.8
		font.family:"微软雅黑"
		elide: Text.ElideRight
	}
	Text{
		id: text_item_val
		text: textData + "cm"
		anchors.top: parent.top
		anchors.topMargin: 8
		anchors.left: parent.left
		anchors.leftMargin: 75
		font.pixelSize: 14
		font.letterSpacing: -0.04
		color:"#FFFFFF";
		opacity: 0.8
		font.family:"微软雅黑"
	}
	MouseArea{
		anchors.fill: parent
		hoverEnabled: true
		onExited: {
			icon_item.width = 4
			icon_item.height = 4
			icon_item.anchors.leftMargin = 10
			icon_item.anchors.topMargin = 14
			icon_item.color = "#FFFFFF"
		}
		onEntered: {									
			icon_item.width = 8
			icon_item.height = 8
			icon_item.anchors.leftMargin = 8
			icon_item.anchors.topMargin = 12
			icon_item.color = "#56C2EB"
			}
		onReleased: {
			//rect_item.color = "#2f3033"
		}
		onPressed: {
			//rect_item.color = "black"
		}
		onClicked: {
			//clickSignal();
			//选中了哪一个item通知给c++
			console.log("DataItem Clicked")
			if (DataModel.currData != DataModel.dataList[index])
			{		
				g_currIndex = -(index+1);
				console.log("data"+g_currIndex)		
				if (!DataModel.currData.isCreate)			
				{
					if ( DataModel.currData.isDirty )
					{							
						saveWin.show()
						saveWin.visible = true
						saveWindow.setText("是否保存当前数据项的更改?", 0)
					}else{
						btn_saveoredit_text.text = "确定"
						DataModel.setCurrDataByUI(DataModel.dataList[index])
						console.log("here")
					}
				}else
				{
					console.log("test removeCurrData")
					//removeCurrData(false);
					g_clickItemNum = -(index+1)	
					removeCurrData()					
					//do something 					
					//DataModel.setCurrDataByUI(DataModel.dataList[index])
				}
			}
		}					
	}
}				


