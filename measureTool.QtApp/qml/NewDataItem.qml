import QtQuick 2.0
import Cloudream.Controls 1.0


Rectangle{
	id: newRect_item 
	width: 134
	height: 32
	
	property var textName: ""
	property var itemImage_s: ""
	property var itemImage_n: ""
	
	signal clickNewSignal(var index);
	
	onClickNewSignal: {
		//console.log("item index " + index)
		//console.log("list Model count " + listModel.count);
		for (var i = 0; i < listModel.count; i++) {
			var item = listView.itemAt(newdata_delegate.width / 2, 16 + 32 * i)
			if (i == index) {
				item.clickOn();
			} else {
				item.reset();
			}
		}
		//清空上面原始数据选中状态
		for (var i = 0; i < listModel_data.count; i++) {
			var item = listView_data.itemAt(newdata_delegate.width / 2, 16 + 32 * i)
			item.reset();
		}
	}
	
	function clickOn() {
		newdata_delegate.color = "black"	//"#2A2B2E"			
	}
	
	function reset() {
		newdata_delegate.color = "#2f3033"
	}
	
	function updateName(name)
	{
		newtext_item.text = name
	}
	
	Rectangle{
		id: newdata_delegate
		width: newRect_item.width
		height: 32
		color: "#2f3033"					
		Image{
			id: newdata_delegate_image
			width: 24
			height: 24
			anchors.left: parent.left
			anchors.leftMargin: 0
			anchors.top: parent.top
			anchors.topMargin: 4
			source: itemImage_n
		}										
		Text { 
			id: newtext_item 
			text: textName 
			width: 80 
			anchors.top: parent.top
			anchors.topMargin: 8
			anchors.left: parent.left
			anchors.leftMargin: 24
			font.pixelSize: 14
			font.letterSpacing: -0.04
			color:"#FFFFFF";
			opacity: 0.8
			font.family:"微软雅黑"
			elide: Text.ElideRight
		}
		MouseArea{
			anchors.fill: parent
			hoverEnabled: true
			onEntered: {													
				//newdata_delegate.color = "#202020"	
				newdata_delegate_image.source = itemImage_s								
			}
			onExited: {					
				//newdata_delegate.color = "#2f3033"
				newdata_delegate_image.source = itemImage_n								
			}
			onReleased: {				
				//newdata_delegate_image.source = itemImage_n
				//newdata_delegate.color = "#2f3033"
			}
			onPressed: {				
				//newdata_delegate_image.source = itemImage_s
				//newdata_delegate.color = "black"
			}
			onClicked: {
				//clickNewSignal()
				if (DataModel.currData != DataModel.newDataList[index])
				{
					g_currIndex = index+1;
					console.log("data"+g_currIndex)
					if (DataModel.currData.saved)
					{
						if ( DataModel.currData.isDirty )
						{							
							saveWin.show()
							saveWin.visible = true
							saveWindow.setText("是否保存当前数据项的更改?", 0)
						}else{
							btn_saveoredit_text.text = "确定"
							DataModel.setCurrDataByUI(DataModel.newDataList[index])
							console.log("here")
						}
					}else
					{
						//removeCurrData(false);
						g_clickItemNum = index+1
						removeCurrData()
						//do something						
						//DataModel.setCurrDataByUI(DataModel.newDataList[index])
					}
					//DataModel.setCurrDataByUI(DataModel.newDataList[index])	
				}					
			}
		}
	}					
}					


