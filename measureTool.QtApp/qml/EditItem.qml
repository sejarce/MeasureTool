import QtQuick.Window 2.0 
import QtQuick 2.7 
import QtQuick.Dialogs 1.2 
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0

import Cloudream.Controls 1.0

Rectangle{
	
	property var textName: ""
	
	function updateDataName(name)
	{
		data_name.text = name
		textName = name
	}

	function myActiveFocus(flag)
	{
		if(flag)
		{
			console.log("ActiveFocus pre")
			data_name.forceActiveFocus()
			data_name.focus = true
			data_name.enabled = true
	
			bgRect.color = "white"
			bgRect.border.color = "green"
			bgRect.opacity = 0.1
			console.log("ActiveFocus")
		}else{
			data_name.focus = false
			data_name.enabled = false
	
			if (DataModel.dataList.length > 0 || DataModel.newDataList.length > 0)
			{
				if(data_name.length == 0)
				{
					//弹框提示 名称不能为空
					console.log("弹框提示 名称不能为空")
					msg_text.text = "名称不能为空"
					msgWin.show()
					msgWin.visible = true
				
					data_name.text = textName
				}
				
				for (var i = 0; i < DataModel.newDataList.length; i++)
				{
					if (DataModel.newDataList[i] == DataModel.currData)
						continue
					
					if ( DataModel.newDataList[i].name == data_name.text)
					{
						//弹框提示 该名称已存在
						console.log("弹框提示 该名称已存在")
						msg_text.text = "该名称已存在"
						msgWin.show()
						msgWin.visible = true
					
						data_name.text = textName
						break
					}
				}
				
				for (var i = 0; i < DataModel.dataList.length; i++)
				{
					if (DataModel.dataList[i] == DataModel.currData)
						continue
					
					if ( DataModel.dataList[i].name == data_name.text)
					{
						//弹框提示 该名称已存在
						console.log("弹框提示 该名称已存在")
						msg_text.text = "该名称已存在"
						msgWin.show()
						msgWin.visible = true
					
						data_name.text = textName
						break
					}
				}

				textName = data_name.text
				root.updateName(data_name.text)
			}
			bgRect.color = "#2A2B2E"
			bgRect.border.color = "#2A2B2E"
			bgRect.opacity = 1.0
		}
	}	
	
	Rectangle {
		id: bgRect
		width: parent.width
		height: parent.height
		color: "#2A2B2E"
		border.color: "#2A2B2E"
		border.width: 1
		opacity: 1.0
	}
	
	TextInput{
		id: data_name
		text: ""
		width: parent.height
		height: parent.width
		anchors.fill: parent

		//acceptableInput: true
		activeFocusOnPress: true
		maximumLength: 10
		echoMode: TextInput.Normal
		
		color: "#FFFFFF"
		font.pixelSize: 12
		font.letterSpacing: -0.04
		font.family:"微软雅黑"
		horizontalAlignment: TextInput.AlignVCenter
		
		focus: true
		
		onAccepted: {
			console.log("onAccepted")
		}

		onEditingFinished:{
			console.log("onEditingFinished")
			data_name.focus = false
			if (DataModel.dataList.length > 0 || DataModel.newDataList.length > 0)
			{
				if(data_name.length == 0)
				{
					//弹框提示 名称不能为空
					console.log("弹框提示 名称不能为空")
					msg_text.text = "名称不能为空"
					msgWin.show()
					msgWin.visible = true
				
					data_name.text = textName
				}
				
				for (var i = 0; i < DataModel.newDataList.length; i++)
				{
					if (DataModel.newDataList[i] == DataModel.currData)
						continue
					
					if ( DataModel.newDataList[i].name == data_name.text)
					{
						//弹框提示 该名称已存在
						console.log("弹框提示 该名称已存在")
						msg_text.text = "该名称已存在"
						msgWin.show()
						msgWin.visible = true
					
						data_name.text = textName
						break
					}
				}
				
				for (var i = 0; i < DataModel.dataList.length; i++)
				{
					if (DataModel.dataList[i] == DataModel.currData)
						continue
					
					if ( DataModel.dataList[i].name == data_name.text)
					{
						//弹框提示 该名称已存在
						console.log("弹框提示 该名称已存在")
						msg_text.text = "该名称已存在"
						msgWin.show()
						msgWin.visible = true
					
						data_name.text = textName
						break
					}
				}

				textName = data_name.text
				root.updateName(data_name.text)
			}
			bgRect.color = "#2A2B2E"
			bgRect.border.color = "#2A2B2E"
			bgRect.opacity = 1.0
		}
		
		MouseArea{
			anchors.fill: parent
			onClicked:{
				console.log("myActiveFocus")
				myActiveFocus(true)
			}
		}
	}
}