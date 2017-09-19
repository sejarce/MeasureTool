import QtQuick.Window 2.0 
import QtQuick 2.7 
import QtQuick.Dialogs 1.2 
import QtQuick.Controls 1.2
import QtQuick.Controls.Styles 1.4
import QtGraphicalEffects 1.0

import Cloudream.Controls 1.0

Rectangle {
    id: root	
    width: 1024		//未包含左右边框都是8
    height: 660		//未包含上边框31，下边框8
	//maximumWidth: Screen.desktopAvailableWidth
	//maximumHeight: Screen.desktopAvailableHeight
	color: "#2f3033"
	
	//qml全局变量区域
	property string g_nowTime: ""
	property bool g_hasModel: false
	property var g_currIndex: 0
	
	property bool fp_clicked: false	
	property var g_floatBarItemMask: 0
	property var g_toolBarItemMask: 0
	property bool g_ctrl_pressed: false
	property bool g_toolbar_enable: false
	
	property bool g_bmsg: false
	property var g_createItem: -1
	property var g_next_msg: 0
	property var g_next_openorclose: -1
	property var g_clickItemNum: 0
	//property var g_data_name_text: ""
	
	signal dataStatusChanged(var sta);

	onDataStatusChanged:{
		console.log(g_currIndex)
		if (sta == 1)
		{
			//保存当前数据
			saveCurrData()
			//切换到新的currData
			console.log("保存当前数据")
			console.log(g_currIndex)
			
			g_bmsg = true
			//if (g_currIndex > 0)
			//{
			//	DataModel.setCurrDataByUI(DataModel.newDataList[g_currIndex-1])
			//}else{
			//	DataModel.setCurrDataByUI(DataModel.dataList[-g_currIndex-1])
			//}
			btn_saveoredit_text.text = "编辑"
			btn_cancelordelete_text.text = "移除"
		}else if (sta == 0)
		{
			//啥也不干
		}else if (sta == -1)
		{
			//不保存当前数据
			cancelCurrData()
			//切换到新的currData
			console.log("不保存当前数据")
			console.log(g_currIndex)
			
			g_bmsg = true
			//if (g_currIndex > 0)
			//{
			//	DataModel.setCurrDataByUI(DataModel.newDataList[g_currIndex-1])
			//}else{
			//	//DataModel.setCurrDataByUI(DataModel.dataList[-g_currIndex-1])
			//}
		}
	}
	
	Timer {
        interval: 500; running: true; repeat: true
        onTriggered: {
			g_nowTime = Qt.formatDateTime(new Date(), "yyyy/MM/dd hh:mm:ss")			
		}
    }
	
	focus: true
	Keys.enabled: true
	Keys.onReleased: {
		switch(event.key)
		{
			case Qt.Key_Control:
			{
				g_ctrl_pressed = false
			}
			break
		}
	}
    Keys.onPressed: {	//基本按键事件处理
		//console.log("key press")
        switch(event.key)
		{
			case Qt.Key_Control:
			{
				g_ctrl_pressed = true
			}
			break
			case Qt.Key_F1:
			{
				if (profileWin.visible)
				{
					profileWin.visible = false
					profileWin.hide()
					console.log("hide")
				}else{
					profileWin.visible = true;
					profileWin.show()
					console.log("show")
				}
				//event.accepted = true;//消息不再往下传				
			}
			break
			case Qt.Key_S:
			{
				if (g_toolbar_enable && g_ctrl_pressed)
				{
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)
								g_next_msg = 2
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					saveFile()
				}
			}
			break
			case Qt.Key_O:
			{
				if (g_toolbar_enable && g_ctrl_pressed)
				{
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)								
								g_next_msg = 1
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					if ( DataModel.isDirty )
					{
						saveWin.show()
						saveWin.visible = true
						saveWindow.setText("打开新文档前 是否 保存当前文档的更改？", 1)								
						return
					}
					
					if (EventWrapper.openFileDlg())
					{
						disableToolBar()
					}
				}
			}
			break
			case Qt.Key_A:
			{
				if (g_toolbar_enable && g_ctrl_pressed)
				{
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)
								g_next_msg = 3
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					EventWrapper.saveasAllData()
				}
			}
        }
    }
	
	//强制保留2位小数
	function toDecimal2(x) { 
      var f = parseFloat(x); 
      if (isNaN(f)) { 
        return false; 
      } 
      var f = Math.round(x*10)/10; 
      var s = f.toString(); 
      var rs = s.indexOf('.'); 
      if (rs < 0) { 
        rs = s.length; 
        s += '.'; 
      } 
      while (s.length <= rs + 1) { 
        s += '0'; 
      } 
      return s; 
    }
	
	function updateName(name)
	{		
		DataModel.currData.setName(name)
		
		for (var i = 0; i < DataModel.newDataList.length; i++)
		{
			if (DataModel.newDataList[i] == DataModel.currData)
			{
				console.log(i)
				var item = listView.itemAt(57, 16 + 32 * i)
				item.updateName(name)
				DataModel.newDataList[i].setName(name)
				break
			}			
		}

		for (var i = 0; i < DataModel.dataList.length; i++)
		{
			if (DataModel.dataList[i] == DataModel.currData)
			{
				var item = listView_data.itemAt(57, 16 + 32 * i)
				item.updateName(name)
				DataModel.dataList[i].setName(name)
				break
			}			
		}
	}
	
	function setCheckItem(index)
	{
		//setCheckToolBar(1)
		console.log("here2")
		
		for (var i = 0; i < listModel_data.count; i++) {
			//强制显示刚创建的item
			//listView_data.positionViewAtIndex(i, ListView.Visible)
			
			var item = listView_data.itemAt(57, 16 + 32 * i)
			if (item != null){
				if (i == index) {
					listView_data.positionViewAtIndex(i, ListView.Visible)
					console.log(item + " index:" + index)
					item.clickOn();	
					item.updateValue(data_val.text)					
				} else {
					item.reset();
				}
			}
		}
		//清空下面原始数据选中状态
		for (var i = 0; i < listModel.count; i++) {
			var item = listView.itemAt(57, 16 + 32 * i)
			if (item != null){
				item.reset()
			}
		}
	}
	
	function setCheckNewItem(index)
	{
		//setCheckToolBar(1)
		
		for (var i = 0; i < listModel.count; i++) {
			//强制显示刚创建的item
			//listView.positionViewAtIndex(i, ListView.Visible)

			var item = listView.itemAt(57, 16 + 32 * i)
			console.log(item)
			if (item != null){
				if (i == index) {
					listView.positionViewAtIndex(i, ListView.Visible)
					item.clickOn();
				} else {
					item.reset();
				}
			}
		}
		//清空上面原始数据选中状态
		for (var i = 0; i < listModel_data.count; i++) {
			var item = listView_data.itemAt(57, 16 + 32 * i)
			if (item != null){
				item.reset()
			}
		}
	}
	
	function showWindow_SaveOrRemove()
	{
		
	}
	
	function showWindow_OkorCancel()
	{
		
	}
	
	function saveCurrData()
	{
		if (DataModel.dataList.length > 0 || DataModel.newDataList.length > 0)
		{
			EventWrapper.saveCurrData()
			DataModel.currData.setSaved(true)
			data_time.text = "上次编辑：" + DataModel.currData.editTime
			image_see.source = "res/icon_see_n.png"
			if (DataModel.currData.type == "高度")
			{
				image_height.source = "res/icon_height_n.png"
			}else if (DataModel.currData.type == "长度")
			{
				image_length.source = "res/icon_length_n.png"					
			}else if (DataModel.currData.type == "表面长度")
			{
				image_facelength.source = "res/icon_facelength_n.png"
			}else if (DataModel.currData.type == "表面围度")
			{
				image_facedim.source = "res/icon_facedim_n.png"
			}else if (DataModel.currData.type == "围度")
			{
				image_dim.source = "res/icon_dim_n.png"
			}			
			btn_saveoredit_text.text = "编辑"
			btn_cancelordelete_text.text = "移除"
			//DataModel.setDirty(true)
		}
	}
	
	function saveFile()
	{
		var itemCount = DataModel.dataList.length + DataModel.newDataList.length
		if ( itemCount > 0 )
		{
			if ( !DataModel.currData.saved )
			{
				removeCurrData()
			}
			else
			{
				saveCurrData()
			}
		}
		
		EventWrapper.saveFile()
		if (DataModel.dataList.length > 0)
		{
			var index = DataModel.dataList.length - 1
			DataModel.setCurrDataByUI(DataModel.dataList[index])
			setCheckItem(index)
		}

		btn_saveoredit_text.text = "编辑"
		btn_cancelordelete_text.text = "移除"
		//DataModel.setDirty(false)
	}
	
	function removeCurrData()
	{
		var flag = 0
		if (DataModel.currData.neworOld)
			flag = 1
		else
			flag = 0

		EventWrapper.removeCurrData()
		
		if(g_clickItemNum != 0)
		{
			if(g_clickItemNum > 0)
			{
				var index = g_clickItemNum-1
				DataModel.setCurrDataByUI(DataModel.newDataList[index])
			}else{
				var index = -g_clickItemNum - 1
				DataModel.setCurrDataByUI(DataModel.dataList[index])
			}
			g_clickItemNum = 0
		}else
		{
			if (flag == 0){
				if (DataModel.dataList.length > 0)
				{
					var index = DataModel.dataList.length - 1
					DataModel.setCurrDataByUI(DataModel.dataList[index])
					setCheckItem(index)
				}					
				else if (DataModel.newDataList.length > 0)
				{
					var index = DataModel.newDataList.length - 1
					DataModel.setCurrDataByUI(DataModel.newDataList[index])
					setCheckNewItem(index)
				}else{
					data_val.text = 0
					//data_name.text = ""
					data_name_item.updateDataName("")
					data_type.text = "类型："
					data_time.text = "上次编辑："
				}
			}else if(flag == 1)
			{
				if (DataModel.newDataList.length > 0)
				{
					var index = DataModel.newDataList.length - 1
					DataModel.setCurrDataByUI(DataModel.newDataList[index])
					setCheckNewItem(index)
				}					
				else if (DataModel.dataList.length > 0)
				{
					var index = DataModel.dataList.length - 1
					DataModel.setCurrDataByUI(DataModel.dataList[index])
					setCheckItem(index)
				}else{
					data_val.text = 0
					//data_name.text = ""
					data_name_item.updateDataName("")
					data_type.text = "类型："
					data_time.text = "上次编辑："
				}
			}
		}
		
		/*
		var num = 0x1
		if(DataModel.currData != null){
			if (DataModel.currData.type == "高度")
			{
				num |= 0x2
			}else if (DataModel.currData.type == "长度")
			{
				num |= 0x4				
			}else if (DataModel.currData.type == "表面长度")
			{
				num |= 0x8
			}else if (DataModel.currData.type == "表面围度")
			{
				num |= 0x10
			}else if (DataModel.currData.type == "围度")
			{
				num |= 0x20
			}
		}
		setCheckToolBar(num)
		*/
		
		btn_saveoredit.enabled = true
		btn_saveoredit.color = "#202020"
		btn_saveoredit_text.text = "编辑"
		btn_cancelordelete_text.text = "移除"
	}
	
	function createNewItem(itemType)
	{		
		var itemCount = DataModel.dataList.length + DataModel.newDataList.length
		
		g_currIndex = DataModel.newDataList.length + 1
		
		btn_export.enabled = false
		image_export.source = "res/icon_export_false.png"
				
		if ( itemCount > 0 )
		{
			if ( !DataModel.currData.saved )
			{
				g_currIndex--
				//removeCurrData(false);
				removeCurrData()
				setCheckNewItem(g_currIndex-1)
			}
			else
			{
				setCheckNewItem(g_currIndex-1)
				if ( DataModel.currData.isDirty )
				{							
					saveWin.show()
					saveWin.visible = true
					saveWindow.setText("是否保存当前数据项的更改？", 0)
					g_createItem = itemType
					console.log(g_createItem)
					return
				}else{
					saveCurrData()
				}
			}
		}		
		EventWrapper.createItem(itemType)
		btn_cancelordelete_text.text = "移除"
		btn_saveoredit.enabled = false
		btn_saveoredit.color = "#2A2B2E"
	}
	
	function cancelCurrData()
	{
		if(DataModel.currData.saved)
		{
			EventWrapper.cancelCurrData()
		}else{
			removeCurrData()
		}
		//DataModel.currData.setSaved(true)
	}	
		
	function disableToolBar()
	{
		btn_see.enabled = false
		image_see.source = "res/icon_see_false.png"
		btn_height.enabled = false
		image_height.source = "res/icon_height_false.png"
		btn_length.enabled = false
		image_length.source = "res/icon_length_false.png"
		btn_facelength.enabled = false
		image_facelength.source = "res/icon_facelength_false.png"
		btn_facedim.enabled = false
		image_facedim.source = "res/icon_facedim_false.png"
		btn_dim.enabled = false
		image_dim.source = "res/icon_dim_false.png"
		
		btn_open.enabled = false
		image_open.source = "res/icon_open_false.png"
		btn_save.enabled = false
		image_save.source = "res/icon_save_false.png"
		btn_saveas.enabled = false
		image_saveas.source = "res/icon_saveas_false.png"
		btn_export.enabled = false
		image_export.source = "res/icon_export_false.png"
		
		btn_undo.enabled = false
		image_undo.source = "res/icon_return_false.png"
		btn_redo.enabled = false
		image_redo.source = "res/icon_redo_false.png"
		
		g_toolbar_enable = false
	}
	
	function enableToolBar()
	{
		btn_see.enabled = true
		image_see.source = "res/icon_see_n.png"
		btn_height.enabled = true
		image_height.source = "res/icon_height_n.png"
		btn_length.enabled = true
		image_length.source = "res/icon_length_n.png"
		btn_facelength.enabled = true
		image_facelength.source = "res/icon_facelength_n.png"
		btn_facedim.enabled = true
		image_facedim.source = "res/icon_facedim_n.png"
		btn_dim.enabled = true
		image_dim.source = "res/icon_dim_n.png"
		
		btn_open.enabled = true
		image_open.source = "res/icon_open_n.png"
		btn_save.enabled = true
		image_save.source = "res/icon_save_n.png"
		btn_saveas.enabled = true
		image_saveas.source = "res/icon_saveas_n.png"
		btn_export.enabled = true
		image_export.source = "res/icon_export_n.png"
		
		airBtn.show()
		g_toolbar_enable = true
	}
	
	function showTips(name,x,y)
	{
		tipWin.visible = true		
		tip_text.text = name
		tipWin.width = name.length*10+10
		tipWin.x = x
		tipWin.y = y		
		tipWin.show()
	}
	
	function hideTips()
	{
		tipWin.visible = false
		tipWin.hide()
	}	
	
	function hideDataWindow()
	{
		ogrewindow.setWidth(mainWindow.width)
		airBtn2.visible = true
		airBtn2.show()
	}
	
	function showDataWindow()
	{
		ogrewindow.setWidth(mainWindow.width - 337)
		airBtn2.visible = false
		airBtn2.hide()
	}
	
	function setCheckToolBar(num)
	{
		g_toolBarItemMask = num
		if (num & 0x1)	//see
		{
			image_see.source = "res/icon_see_s.png"
		}else
		{
			image_see.source = "res/icon_see_n.png"
		}
		if (num & 0x2)	//height
		{
			image_height.source = "res/icon_height_s.png"
		}else
		{
			image_height.source = "res/icon_height_n.png"
		}
		if (num & 0x4)	//length
		{
			image_length.source = "res/icon_length_s.png"
		}else
		{
			image_length.source = "res/icon_length_n.png"
		}
		if (num & 0x8)	//curvelength
		{
			image_facelength.source = "res/icon_facelength_s.png"
		}else
		{
			image_facelength.source = "res/icon_facelength_n.png"
		}
		if (num & 0x20)	//dim
		{
			image_dim.source = "res/icon_dim_s.png"
		}else
		{
			image_dim.source = "res/icon_dim_n.png"
		}
		if (num & 0x10)	//ConvexDim
		{
			image_facedim.source = "res/icon_facedim_s.png"
		}else
		{
			image_facedim.source = "res/icon_facedim_n.png"
		}
	}
	
	function setCheckFloatBar(num)
	{
		g_floatBarItemMask = num
		if (num & 0x1)	//showall
		{
			image_allsee.source = "res/icon_allsee_s.png"
			allsee.color = "#202020"
		}else
		{
			image_allsee.source = "res/icon_allsee_n.png"
			allsee.color = "#2f3033"
		}
		if (num & 0x2)	//camera reset
		{
			
		}
		if (num & 0x4)	//show/edit featurepoint
		{
			
		}
	}
	/*
	Text {
		id: time 
		
		width: parent.width
		height: 32
		anchors.top: parent.top
		anchors.topMargin: 8
		anchors.left: parent.left
		anchors.leftMargin: 216
		font.pixelSize: 16
		color:"#FFFFFF";
		font.family:"微软雅黑"
	}
	*/
	
	//title
	Image{
		width: 24
		height: 24
		anchors.top: parent.top
		anchors.topMargin: 8
		anchors.left: parent.left
		anchors.leftMargin: 8
		source: "res/icon_app.png"
	}
	
	Text
	{
		text: "量体助手Pro"
		width: parent.width
		height: 32
		anchors.top: parent.top
		anchors.topMargin: 8
		anchors.left: parent.left
		anchors.leftMargin: 40
		font.pixelSize: 16
		color:"#FFFFFF";
		font.family:"微软雅黑"
	}
	
	MouseArea {
		id: dragRegion
        anchors.fill: parent
        property point clickPos: "0,0"
        onPressed: {
            clickPos  = Qt.point(mouse.x,mouse.y)
            }
        onPositionChanged: {
			if (clickPos.y < 32)	//选中标题栏方可拖动,标题栏宽度为32
			{
				//鼠标偏移量
				var delta = Qt.point(mouse.x-clickPos.x, mouse.y-clickPos.y)
				//如果mainwindow继承自QWidget,用setPos
				mainWindow.setX(mainWindow.x+delta.x)
				mainWindow.setY(mainWindow.y+delta.y)
			}
        }
		onDoubleClicked: {
			if (clickPos.y < 32)	//选中标题栏方可拖动,标题栏宽度为32
			{
				airBtn2.show()
				if (mainWindow.width < Screen.desktopAvailableWidth && mainWindow.height < Screen.desktopAvailableHeight)
				{
					console.log("showMaximized")
					image_max.source = "res/wmini.png"
					mainWindow.showMaximized()
				}else{
					console.log("showNormal")
					image_max.source = "res/wmac.png"
					mainWindow.showNormal()
				}
			}
		}
	}
	
	//close min max	
	Row{
		anchors.right: parent.right
		anchors.rightMargin: 0
		anchors.top: parent.top
		anchors.topMargin: 0
		spacing: 1
		//min_btn
		Rectangle {
			id: min_btn
			width: 48
			height: 32		
			color: "#2f3033"
			
			Image {
				id: image_min
				anchors.fill:parent;
				source: min_btn.enabled ? "res/minimize.png" : ""
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button min enter")
					//image_min.source = "res/minimize.png"
					min_btn.color = "#202020"
				}
				onExited: {
					console.log("button min exit")
					//image_min.source = "res/minimize.png"
					min_btn.color = "#2f3033"
				}
				onReleased: {
					console.log("button min release")
					//image_min.source = "res/minimize.png"
					min_btn.color = "#2f3033"
				}
				onPressed: {
					console.log("button min press")
					//image_min.source = "res/minimize.png"
					min_btn.color = "black"
				}
				onClicked: {
					console.log("button min click")
					mainWindow.showMinimized()
				}
			}
		}
		//max_btn
		Rectangle {
			id: max_btn
			width: 48
			height: 32		
			color: "#2f3033"
			
			Image {
				id: image_max
				anchors.fill:parent;
				source: max_btn.enabled ? "res/wmac.png" : ""
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button max enter")
					//image_max.source = "res/wmac.png"
					max_btn.color = "#202020"
				}
				onExited: {
					console.log("button max exit")
					//image_max.source = "res/wmac.png"
					max_btn.color = "#2f3033"
				}
				onReleased: {
					console.log("button max release")
					//image_max.source = "res/wmac.png"
					max_btn.color = "#2f3033"
				}
				onPressed: {
					console.log("button max press")
					//image_max.source = "res/wmac.png"
					max_btn.color = "black"
				}
				onClicked: {
					console.log("button max click")
					airBtn2.show()					
					if (mainWindow.width < Screen.desktopAvailableWidth && mainWindow.height < Screen.desktopAvailableHeight)
					{
						console.log("showMaximized")
						image_max.source = "res/wmini.png"	
						mainWindow.showMaximized()
					}else{
						console.log("showNormal")
						image_max.source = "res/wmac.png"				
						mainWindow.showNormal()
					}					
					//showDataWindow()
				}
			}
		}
		//quit_btn
		Rectangle {
			id: quit_btn
			width: 48
			height: 32		
			color: "#2f3033"
			
			Image {
				id: image_quit
				anchors.fill:parent;
				source: quit_btn.enabled ? "res/close.png" : ""
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button quit enter")
					//image_quit.source = "res/close.png"
					quit_btn.color = "#202020"
				}
				onExited: {
					console.log("button quit exit")
					//image_quit.source = "res/close.png"
					quit_btn.color = "#2f3033"
				}
				onReleased: {
					console.log("button quit release")
					//image_quit.source = "res/close.png"
					quit_btn.color = "#2f3033"
				}
				onPressed: {
					console.log("button quit press")
					//image_quit.source = "res/close.png"
					quit_btn.color = "black"
				}
				onClicked: {
					console.log("button quit click")
					
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)								
								g_next_msg = 4
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					if ( DataModel.isDirty )
					{
						saveWin.show()
						saveWin.visible = true
						saveWindow.setText("关闭前 是否 保存当前文档的更改？", 2)								
						return
					}
					
					mainWindow.close()
				}
			}
		}
	}

	//open save saveas export
	Row{
		anchors.left: parent.left
		anchors.leftMargin: 19
		anchors.top: parent.top
		anchors.topMargin: 56
		spacing: 4
		
		//open
		Rectangle {
			id: btn_open
			width: 48
			height: 48		
			color: "#2f3033"
			
			Image {
				id: image_open
				anchors.fill:parent;
				source: btn_open.enabled ? "res/icon_open_n.png" : ""
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button open enter")
					image_open.source = "res/icon_open_f.png"
					
					showTips("打开",26,100)
				}
				onExited: {
					console.log("button open exit")
					image_open.source = "res/icon_open_n.png"
					
					hideTips()
				}
				onReleased: {
					console.log("button open release")
					image_open.source = "res/icon_open_n.png"
					btn_open.color = "#2f3033"
				}
				onPressed: {
					console.log("button open press")
					image_open.source = "res/icon_open_s.png"
					btn_open.color = "black"
				}
				onClicked: {
					console.log("button open click")
					
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)								
								g_next_msg = 1
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					if ( DataModel.isDirty )
					{
						saveWin.show()
						saveWin.visible = true
						saveWindow.setText("打开新文档前 是否 保存当前文档的更改？", 1)								
						return
					}
					
					if (EventWrapper.openFileDlg())
					{
						disableToolBar()
					}					
				}
			}
		}
		//save
		Rectangle {
			id: btn_save
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_save
				anchors.fill:parent;
				source: btn_save.enabled ? "res/icon_save_n.png" : "res/icon_save_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button save enter")
					image_save.source = "res/icon_save_f.png"
					
					showTips("保存",78,100)
				}
				onExited: {
					console.log("button save exit")
					image_save.source = "res/icon_save_n.png"
					
					hideTips()
				}
				onReleased: {
					console.log("button save release")
					image_save.source = "res/icon_save_n.png"
					btn_save.color = "#2f3033"
				}
				onPressed: {
					console.log("button save press")
					image_save.source = "res/icon_save_s.png"
					btn_save.color = "black"
				}
				onClicked: {
					console.log("button save click")
					
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)
								g_next_msg = 2
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					saveFile()
				}
			}
		}
		//saveas
		Rectangle {
			id: btn_saveas
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			Image {
				id: image_saveas
				anchors.fill:parent;
				source: btn_saveas.enabled ? "res/icon_saveas_n.png" : "res/icon_saveas_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button saveas enter")
					image_saveas.source = "res/icon_saveas_f.png"
					
					showTips("另存为",130,100)
				}
				onExited: {
					console.log("button saveas exit")
					image_saveas.source = "res/icon_saveas_n.png"
					
					hideTips()
				}
				onReleased: {
					console.log("button saveas release")
					image_saveas.source = "res/icon_saveas_n.png"
					btn_saveas.color = "#2f3033"
				}
				onPressed: {
					console.log("button saveas press")
					image_saveas.source = "res/icon_saveas_s.png"
					btn_saveas.color = "black"
				}
				onClicked: {
					console.log("button saveas click")
					
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)
								g_next_msg = 3
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					EventWrapper.saveasAllData()
				}
			}
		}	
		//export
		Rectangle {
			id: btn_export
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_export
				anchors.fill:parent;
				source: btn_export.enabled ? "res/icon_export_n.png" : "res/icon_export_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button export enter")
					image_export.source = "res/icon_export_f.png"
					
					showTips("导出",182,100)
				}
				onExited: {
					console.log("button export exit")
					image_export.source = "res/icon_export_n.png"
					
					hideTips()
				}
				onReleased: {
					console.log("button export release")
					image_export.source = "res/icon_export_n.png"
					btn_export.color = "#2f3033"
				}
				onPressed: {
					console.log("button export press")
					image_export.source = "res/icon_export_s.png"
					btn_export.color = "black"
				}
				onClicked: {
					console.log("button export click")
					
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)
								g_next_msg = 5
								return
							}else{
								saveCurrData()
							}
						}
					}
					
					EventWrapper.exportData()
				}
			}
		}		
	}
	
	//redo undo
	Row{
		anchors.left: parent.left
		anchors.leftMargin: 260
		anchors.top: parent.top
		anchors.topMargin: 56
		spacing: 4
		//undo
		Rectangle {
			id: btn_undo
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_undo
				anchors.fill:parent;
				source: btn_undo.enabled ? "res/icon_return_n.png" : "res/icon_return_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button return enter")
					image_undo.source = "res/icon_return_f.png"
					
					showTips("撤销", 265,100)
				}
				onExited: {
					console.log("button return exit")
					image_undo.source = "res/icon_return_n.png"
					
					hideTips()
				}
				onReleased: {
					console.log("button return release")
					image_undo.source = "res/icon_return_n.png"
					btn_undo.color = "#2f3033"
				}
				onPressed: {
					console.log("button return press")
					image_undo.source = "res/icon_return_s.png"
					btn_undo.color = "black"
				}
				onClicked: {
					console.log("button return click")
				}
			}
		}
		//redo	
		Rectangle {
			id: btn_redo
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_redo
				anchors.fill:parent;
				source: btn_redo.enabled ? "res/icon_redo_n.png" : "res/icon_redo_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button redo enter")
					image_redo.source = "res/icon_redo_f.png"
					
					showTips("恢复",327,100)
				}
				onExited: {
					console.log("button redo exit")
					image_redo.source = "res/icon_redo_n.png"
					
					hideTips()
				}
				onReleased: {
					console.log("button redo release")
					image_redo.source = "res/icon_redo_n.png"
					btn_redo.color = "#2f3033"
				}
				onPressed: {
					console.log("button redo press")
					image_redo.source = "res/icon_redo_s.png"
					btn_redo.color = "black"
				}
				onClicked: {
					console.log("button redo click")
				}
			}
		}		
	}
	
	//具体业务
	Row{
		anchors.left: parent.left
		anchors.leftMargin: 400
		anchors.top: parent.top
		anchors.topMargin: 56
		spacing: 8
		
		//see
		Rectangle {
			id: btn_see
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_see
				anchors.fill:parent;
				source: btn_see.enabled ? "res/icon_see_n.png" : "res/icon_see_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button see enter")
					btn_see.color = "black"
					
					showTips("浏览模式", 405,100)
				}
				onExited: {
					console.log("button see exit")
					btn_see.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
				}
				onPressed: {
				}
				onClicked: {
					var itemCount = DataModel.dataList.length + DataModel.newDataList.length
					if ( itemCount > 0 )
					{
						if ( !DataModel.currData.saved )
						{
							removeCurrData();
						}
						else
						{
							if ( DataModel.currData.isDirty )
							{							
								saveWin.show()
								saveWin.visible = true
								saveWindow.setText("是否保存当前数据项的更改？", 0)
							}else{
								saveCurrData()
							}
						}
					}
					setCheckToolBar(1)
				}
			}
		}
		//height	
		Rectangle {
			id: btn_height
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_height
				anchors.fill:parent;
				source: btn_height.enabled ? "res/icon_height_n.png" : "res/icon_height_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button height enter")
					btn_height.color = "black"
					
					showTips("高度", 461,100)
				}
				onExited: {
					console.log("button height exit")
					btn_height.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
				}
				onPressed: {
				}
				onClicked: {
					console.log("button height click")					
					createNewItem(0);	
					setCheckToolBar(2)
					//image_height.source = "res/icon_height_s.png"
				}
			}
		}
		//length	
		Rectangle {
			id: btn_length
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_length
				anchors.fill:parent;
				source: btn_length.enabled ? "res/icon_length_n.png" : "res/icon_length_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button length enter")
					btn_length.color = "black"
					
					showTips("长度", 517,100)
				}
				onExited: {
					console.log("button length exit")
					btn_length.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {				
				}
				onPressed: {				
				}
				onClicked: {
					console.log("button length click")
					createNewItem(1)
					setCheckToolBar(4)
					//image_length.source = "res/icon_length_s.png"
				}
			}
		}
		//facelength	
		Rectangle {
			id: btn_facelength
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			Image {
				id: image_facelength
				anchors.fill:parent;
				source: btn_facelength.enabled ? "res/icon_facelength_n.png" : "res/icon_facelength_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button facelength enter")
					btn_facelength.color = "black"
					
					showTips("表面长度", 573,100)
				}
				onExited: {
					console.log("button facelength exit")
					btn_facelength.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
				}
				onPressed: {
				}
				onClicked: {
					console.log("button facelength click")
					createNewItem(2)
					setCheckToolBar(8)
					//image_facelength.source = "res/icon_facelength_s.png"
				}
			}
		}
		//facedim	
		Rectangle {
			id: btn_facedim
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_facedim
				anchors.fill:parent;
				source: btn_facedim.enabled ? "res/icon_facedim_n.png" : "res/icon_facedim_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button facedim enter")
					btn_facedim.color = "black"
					
					showTips("表面围度", 629,100)
				}
				onExited: {
					console.log("button facedim exit")
					btn_facedim.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
				}
				onPressed: {
				}
				onClicked: {
					console.log("button facedim click")
					createNewItem(3)
					setCheckToolBar(16)
					//image_facedim.source = "res/icon_facedim_s.png"
				}
			}
		}
		//dim	
		Rectangle {
			id: btn_dim
			width: 48
			height: 48		
			color: "#2f3033"
			enabled: false
			
			Image {
				id: image_dim
				anchors.fill:parent;
				source: btn_dim.enabled ? "res/icon_dim_n.png" : "res/icon_dim_false.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button dim enter")
					btn_dim.color = "black"
					
					showTips("围度", 685,100)
				}
				onExited: {
					console.log("button dim exit")
					btn_dim.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
				}
				onPressed: {
				}
				onClicked: {
					console.log("button dim click")
					createNewItem(4)
					setCheckToolBar(32)
					//image_dim.source = "res/icon_dim_s.png"
				}
			}
		}
	}
	
	//help
	Row{
		anchors.right: parent.right
		anchors.rightMargin: 16
		anchors.top: parent.top
		anchors.topMargin: 56
		spacing: 8
		//help
		Rectangle {
			id: btn_help
			width: 48
			height: 48		
			color: "#2f3033"
			
			Image {
				id: image_help
				anchors.fill:parent;
				source: btn_help.enabled ? "res/icon_help_n.png" : ""
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button help enter")
					btn_help.color = "black"
					
					showTips("帮助", mainWindow.width - 60,100)
				}
				onExited: {
					console.log("button help exit")
					btn_help.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
				}
				onPressed: {
					console.log("buttonhelp press")
					image_help.source = "res/icon_help_s.png"
				}
				onClicked: {
					console.log("button help click")
				}
			}
		}
	}
	
	Rectangle{
		id: dataRectangle
		objectName: "dataRect"
		x: root.x + root.width - 337;
		y: root.y + 108;
		width: 337;
		height: root.height - 108;
		color: "#999999"//Qt.rgba(Math.random(), Math.random(), Math.random(), 1.0);
		visible: true
		Image{
			anchors.fill:parent;
			source: "res/bg.png"
		}
	}
	
	Rectangle{
		id: dataWindow
		objectName: "Datawindow"
		x: dataRectangle.x	
		y: dataRectangle.y + 30				
		width: dataRectangle.width - 16					
		height: dataRectangle.height - 70		
		color: "#2A2B2E"//Qt.rgba(Math.random(), Math.random(), Math.random(), 1.0);
		visible: true
	}	
	
	//数据信息标题栏
	Rectangle{
		id: dataTitle
		objectName: "DataTitle"
		x: dataWindow.x	
		y: dataWindow.y			
		width: dataWindow.width				
		height: 32
		color: "#2f3033"//Qt.rgba(Math.random(), Math.random(), Math.random(), 1.0);
		visible: true;
		Text{
			text: "数据信息"
			width: parent.width
			height: 32
			anchors.top: parent.top
			anchors.topMargin: 8
			anchors.left: parent.left
			anchors.leftMargin: 16
			font.pixelSize: 14
			font.letterSpacing: 0
			color:"#FFFFFF";
			opacity: 0.8
			font.family:"微软雅黑"
		}
		
		Rectangle{
			id: close_datawindow
			objectName: "Close_Datawindow"
			width: 24			
			height: 24
			anchors.top: parent.top
			anchors.topMargin: 4
			anchors.right: parent.right
			anchors.rightMargin: 4
			color: "#2f3033"//Qt.rgba(Math.random(), Math.random(), Math.random(), 1.0);
			visible: true;
			Image {
				id: image_close
				anchors.fill: parent;
				source: "res/icon_off_n.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button close enter")
					image_close.source = "res/icon_off_s.png"					
					close_datawindow.color = "#202020"
				}
				onExited: {
					console.log("button close exit")
					image_close.source = "res/icon_off_n.png"						
					close_datawindow.color = "#2f3033"
				}
				onReleased: {
					console.log("button close release")					
					close_datawindow.color = "#2f3033"
				}
				onPressed: {
					console.log("button close press")				
					close_datawindow.color = "black"
				}
				onClicked: {
					console.log("button close click")	
					ogrewindow.setWidth(mainWindow.width);
					//ogrewindow.width = mainWindow.width
					//ogrewindow.maximumWidth = Screen.desktopAvailableWidth
					airBtn2.visible = true
					airBtn2.show()
				}
			}
		}
	}
	
	//左边数据rect
	Rectangle{
		id: leftDataRect
		objectName: "LeftDataRect"
		x: dataWindow.x	
		y: dataWindow.y + dataTitle.height + 1 			
		width: 150			
		height: (dataWindow.height - dataTitle.height - 1)*0.6
		color: "#2f3033"
		visible: true;
		
		Rectangle{			
			width: leftDataRect.width//-5				
			height: leftDataRect.height - 32
			anchors.top: parent.top
			anchors.topMargin: 32
			anchors.left: parent.left
			anchors.leftMargin: 0
			color: "#2f3033"
			visible: true; 
	
			Component {
				id: list_data_delegate
				DataItem {
					textName: tName
					textData: tData
				}
			}
			
			ListModel{
				id: listModel_data
			}
			
			ScrollView{
				anchors.fill: parent
				//style: style:ScrollViewStyle{					
				//}	
				highlightOnFocus: true
				horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff
				verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
					
				ListView{
					id: listView_data
					model: listModel_data
					anchors.fill: parent
					delegate: list_data_delegate
					snapMode: ListView.SnapOneItem 
					orientation: ListView.Vertical
					cacheBuffer: 32*100
				}
			}
		}
		
		Text{
			text: "基本数据"
			anchors.top: parent.top
			anchors.topMargin: 8
			anchors.left: parent.left
			anchors.leftMargin: 8
			font.pixelSize: 12
			font.letterSpacing: -0.03
			color:"#FFFFFF";
			opacity: 0.5
			font.family:"微软雅黑"
		}
	}
	
	Connections {
		target: DataModel
		onDataReady: {
			console.log("get dataReady");
			listModel_data.clear();
			for (var i = 0; i < DataModel.dataList.length; i++) {
				listModel_data.append({
					tName: DataModel.dataList[i].name,
					tData: toDecimal2(DataModel.dataList[i].value * 100.0),
					tType: DataModel.dataList[i].type
				})
			}
			
			//basedata_scroll.updateBubble(DataModel.dataList.length*32.0)
			
			btn_saveoredit_text.text = "确定"
			btn_cancelordelete_text.text = "取消"
		}
	}
	
	//左下数据rect
	Rectangle{
		id: leftbottomDataRect
		objectName: "LeftbottomDataRect"
		x: leftDataRect.x	
		y: leftDataRect.y + leftDataRect.height 			
		width: leftDataRect.width				
		height: leftDataRect.height * 0.6667
		color: "#2f3033"
		visible: true
		
		Rectangle{			
			width: leftDataRect.width//-5				
			height: leftDataRect.height * 0.6667 - 32
			anchors.top: parent.top
			anchors.topMargin: 32
			anchors.left: parent.left
			anchors.leftMargin: 0
			color: "#2f3033"
			visible: true; 
			//定义一个通用组件:	新增数据组件			
			Component {
				id: list_newdata_delegate
				NewDataItem {
					textName: tName
					itemImage_s: tImage_s
					itemImage_n: tImage_n
				}
			}
			ListModel {
				id: listModel	
			}	

			ScrollView{
				anchors.fill: parent
				//style: style:ScrollViewStyle{					
				//}
				highlightOnFocus: true
				horizontalScrollBarPolicy: Qt.ScrollBarAlwaysOff
				verticalScrollBarPolicy: Qt.ScrollBarAsNeeded
				
				ListView{
					id: listView
					model: listModel
					anchors.fill: parent
					delegate: list_newdata_delegate
					snapMode: ListView.SnapOneItem
					orientation: ListView.Vertical
					cacheBuffer: 32*100
				}
			}
		}
	
		Text{
			text: "新增数据"
			anchors.top: parent.top
			anchors.topMargin: 8
			anchors.left: parent.left
			anchors.leftMargin: 8
			font.pixelSize: 12
			font.letterSpacing: -0.03
			color:"#FFFFFF";
			opacity: 0.5
			font.family:"微软雅黑"
		}
	}
	
	Connections {
		target: DataModel
		onDataReady: {
			console.log("get dataReady");
			listModel.clear();
			for (var i = 0; i < DataModel.newDataList.length; i++) {
				console.log("new newdata")
				listModel.append({
					tImage_n: DataModel.newDataList[i].image_n,
					tImage_s: DataModel.newDataList[i].image_s,
					tName: DataModel.newDataList[i].name,
					tData: toDecimal2(DataModel.newDataList[i].value * 100.0),
					tType: DataModel.newDataList[i].type
				})
				
			}
			
			//newdata_scroll.updateBubble(DataModel.newDataList.length*32.0)
			
			btn_saveoredit_text.text = "确定"
			btn_cancelordelete_text.text = "取消"
		}
	}
	
	//右边数据显示区域
	Rectangle{
		id: rightDataRect
		objectName: "RightDataRect"
		x: leftDataRect.x + 150	
		y: leftDataRect.y 			
		width: 172				
		height: leftDataRect.height
		color: "#2A2B2E"
		visible: true;
		
		Text{
			id: data_val
			text: "0"
			anchors.top: parent.top
			anchors.topMargin: 50
			anchors.left: parent.left
			anchors.leftMargin: 16
			font.pixelSize: 36			
			font.letterSpacing: -3.0
			lineHeight: 50
			color:"#56c2EA";
			font.family:"微软雅黑"
		}

		Text{
			id: data_cm
			text: "cm"
			anchors.top: parent.top
			anchors.topMargin: 63
			anchors.left: parent.left
			anchors.leftMargin: 100
			font.pixelSize: 20
			font.letterSpacing: 0
			lineHeight: 28
			color:"#56c2EA";
			font.family:"微软雅黑"
		}
		
		Text{
			id: data_name0
			text: "名称：" 
			anchors.top: parent.top
			anchors.topMargin: 130
			anchors.left: parent.left
			anchors.leftMargin: 17
			font.pixelSize: 12
			font.letterSpacing: -0.04
			lineHeight: 14
			color:"#ffffff";
			font.family:"微软雅黑"
		}
		
		EditItem {
			id: data_name_item
			width: 120
			height: 24
			anchors.top: parent.top
			anchors.topMargin: 130
			anchors.left: parent.left
			anchors.leftMargin: 50
			color: "#2A2B2E"
		}
		
		Text{
			id: data_type
			text: "类型：" 
			anchors.top: parent.top
			anchors.topMargin: 155
			anchors.left: parent.left
			anchors.leftMargin: 17
			font.pixelSize: 12
			font.letterSpacing: -0.04
			lineHeight: 14
			color:"#ffffff";
			font.family:"微软雅黑"
		}
		
		Text{
			id: data_time
			text: "上次编辑：" 
			anchors.top: parent.top
			anchors.topMargin: 180
			anchors.left: parent.left
			anchors.leftMargin: 17
			font.pixelSize: 10
			font.letterSpacing: -0.04
			lineHeight: 14
			color:"#ffffff";
			font.family:"微软雅黑"
		}
		
		/*
		Rectangle{
			id: btn_edit
			width: 24
			height: 24
			anchors.top: parent.top
			anchors.topMargin: 124
			anchors.left: parent.left
			anchors.leftMargin: 119
			color: "#2A2B2E"
			visible: false
			Image{
				id: btn_edit_image
				anchors.fill: parent
				source: "res/icon_adddata_name_edit_n.png"
			}
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					btn_edit.color = "#202020"	
				}
				onExited: {
					btn_edit.color = "#2A2B2E"	
				}
				onReleased: {				
					btn_edit_image.source = "res/icon_adddata_name_edit_n.png"
					btn_edit.color = "#2A2B2E"
				}
				onPressed: {
					btn_edit_image.source = "res/icon_adddata_name_edit_s.png"
					btn_edit.color = "black"
				}
				onClicked: {
					console.log("AAAAAAAAAAAAAAAAAAA");
				}
			}
		}
		*/
		
		//保存or编辑
		Rectangle{
			id: btn_saveoredit
			width: 130
			height: 20
			anchors.top: parent.top
			anchors.topMargin: rightDataRect.height
			anchors.left: parent.left
			anchors.leftMargin: 23
			color: "#202020"
			opacity: 0.8			
			border.width: 0.5
			border.color: "#101010"
			visible: true;
			
			Text{
				id: btn_saveoredit_text 
				text: "编辑"
				anchors.top: parent.top
				anchors.topMargin: 3
				anchors.left: parent.left
				anchors.leftMargin: 55
				font.pixelSize: 10
				font.letterSpacing: 0
				lineHeight: 14
				color: "#ffffff";
				font.family:"微软雅黑"
			}
			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					btn_saveoredit.border.width = 2.0	
				}
				onExited: {
					btn_saveoredit.border.width = 0.5
				}
				onReleased: {				
					btn_saveoredit.color = "#202020"
				}
				onPressed: {
					btn_saveoredit.color = "black"
				}
				onClicked: {
					if (DataModel.currData == null)
					{
						console.log("currData,null")
					}else{				
						if (btn_saveoredit_text.text == "确定")
						{	//save
							if ( !DataModel.currData.isCreate )
							{
								//data_name_item.myActiveFocus(false)

								saveCurrData()
								DataModel.currData.setSaved(true)
							
								setCheckToolBar(1)
							
								btn_saveoredit_text.text = "编辑"
								btn_cancelordelete_text.text = "移除"
							}
						}else{	//edit
							//通知3D进入编辑模式
							if (DataModel.currData.type == "高度")
							{
								//通知3d进入高度编辑模式
								setCheckToolBar(2)
								EventWrapper.activeEditMode(0)
							}else if (DataModel.currData.type == "长度")
							{
								setCheckToolBar(4)
								EventWrapper.activeEditMode(1)							
							}else if (DataModel.currData.type == "表面长度")
							{
								setCheckToolBar(8)
								EventWrapper.activeEditMode(2)
							}else if (DataModel.currData.type == "表面围度")
							{
								setCheckToolBar(16)
								EventWrapper.activeEditMode(3)
							}else if (DataModel.currData.type == "围度")
							{
								setCheckToolBar(32)
								EventWrapper.activeEditMode(4)
							}
							//DataModel.currData.setSaved(false)
							btn_saveoredit_text.text = "确定"
							btn_cancelordelete_text.text = "取消"
						}	
					}					
				}
			}
		}				
		//取消or移除
		Rectangle{
			id: btn_cancelordelete
			width: 130
			height: 20
			anchors.top: parent.top
			anchors.topMargin: rightDataRect.height+28
			anchors.left: parent.left
			anchors.leftMargin: 23
			color: "#202020"
			opacity: 0.8			
			border.width: 0.5
			border.color: "#101010"
			visible: true;
			
			Text{
				id: btn_cancelordelete_text
				text: "移除"
				anchors.top: parent.top
				anchors.topMargin: 3
				anchors.left: parent.left
				anchors.leftMargin: 55
				font.pixelSize: 10
				font.letterSpacing: 0
				lineHeight: 14
				color: "#ffffff";
				font.family:"微软雅黑"
			}
			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					btn_cancelordelete.border.width = 2.0	
				}
				onExited: {
					btn_cancelordelete.border.width = 0.5
				}
				onReleased: {				
					btn_cancelordelete.color = "#202020"
				}
				onPressed: {
					btn_cancelordelete.color = "black"
				}
				onClicked: {
					if (DataModel.currData == null)
					{
						console.log("currData,null")
					}else{
						if (btn_cancelordelete_text.text == "移除")
						{
							console.log("remove")
							if ( !DataModel.currData.saved )
							{
								removeCurrData()
								setCheckToolBar(1)
							}else{
								deleteWin.show()
								deleteWin.visible = true
							}
						}else{
							//console.log("cancel")
							cancelCurrData()
							setCheckToolBar(1)
						}	
					}					
				}
			}
		}		
	}
	

	OgreWindow{
		id: ogrewindow;
		objectName: "ogrewindow0"
		x: root.x;
		y: root.y + 108; 
		width: root.width //- 337;
		height: root.height - 108;
		maximumWidth: Screen.desktopAvailableWidth - 337
		maximumHeight: Screen.desktopAvailableHeight -108
		color: "red";
		flags: Qt.FramelessWindowHint
		visible: false;
		
		MouseArea{
			anchors.fill: parent
			onClicked: {
				//console.log("00")
				//ogrewindow.lower();
			}
			
			onWidthChanged:{
				console.log(ogrewindow.width)
				console.log("ogrewindow changed width " + ogrewindow.width);
			}
		}
		
		Rectangle{
			anchors.fill: parent
			
			focus: true
			Keys.enabled: true
			Keys.onReleased: {
				switch(event.key)
				{
					case Qt.Key_Control:
					{
						g_ctrl_pressed = false
					}
					break
				}
			}
			Keys.onPressed: {	//基本按键事件处理
				//console.log("key press")
				switch(event.key)
				{
					case Qt.Key_Control:
					{
						g_ctrl_pressed = true
					}
					break
					case Qt.Key_F1:
					{
						if (profileWin.visible)
						{
							profileWin.visible = false
							profileWin.hide()
							console.log("hide")
						}else{
							profileWin.visible = true;
							profileWin.show()
							console.log("show")
						}
						//event.accepted = true;//消息不再往下传				
					}
					break
					case Qt.Key_S:
					{
						if (g_toolbar_enable && g_ctrl_pressed)
						{
							var itemCount = DataModel.dataList.length + DataModel.newDataList.length
							if ( itemCount > 0 )
							{
								if ( !DataModel.currData.saved )
								{
									removeCurrData();
								}
								else
								{
									if ( DataModel.currData.isDirty )
									{							
										saveWin.show()
										saveWin.visible = true
										saveWindow.setText("是否保存当前数据项的更改？", 0)
										g_next_msg = 2
										return
									}else{
										saveCurrData()
									}
								}
							}
					
							saveFile()
						}
					}
					break
					case Qt.Key_O:
					{
						var itemCount = DataModel.dataList.length + DataModel.newDataList.length
						if ( itemCount > 0 )
						{
							if ( !DataModel.currData.saved )
							{
								removeCurrData();
							}
							else
							{
								if ( DataModel.currData.isDirty )
								{							
									saveWin.show()
									saveWin.visible = true
									saveWindow.setText("是否保存当前数据项的更改？", 0)								
									g_next_msg = 1
									return
								}else{
									saveCurrData()
								}
							}
						}
					
						if ( DataModel.isDirty )
						{
							saveWin.show()
							saveWin.visible = true
							saveWindow.setText("打开新文档前 是否 保存当前文档的更改？", 1)								
							return
						}
						
						if (EventWrapper.openFileDlg())
						{
							disableToolBar()
						}
					}
					break
					case Qt.Key_A:
					{
						if (g_ctrl_pressed && g_toolbar_enable)
						{
							var itemCount = DataModel.dataList.length + DataModel.newDataList.length
							if ( itemCount > 0 )
							{
								if ( !DataModel.currData.saved )
								{
									removeCurrData();
								}
								else
								{
									if ( DataModel.currData.isDirty )
									{							
										saveWin.show()
										saveWin.visible = true
										saveWindow.setText("是否保存当前数据项的更改？", 0)
										g_next_msg = 3
										return
									}else{
										saveCurrData()
									}
								}
							}
					
							EventWrapper.saveasAllData()
						}
					}
				}
			}
		}
	}
	
	AirBtn{
		id: airBtn
		objectName: "airbtn"
		x: ogrewindow.x + 14
		y: ogrewindow.y + 22
		width: 48
		height: 146
		color: "black"
		flags: Qt.FramelessWindowHint
		visible: false
		
		Rectangle{
			id: allsee
			width: 48
			height: 48
			anchors.top: parent.top
			anchors.topMargin: 0
			anchors.left: parent.left
			anchors.leftMargin: 0
			color: "#2f3033"
			Image {
				id: image_allsee
				anchors.fill:parent;
				source: "res/icon_allsee_n.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					//console.log("button allsee enter")
					//image_allsee.source = "res/icon_allsee_s.png"
					//allsee.color = "#202020"
					if( g_floatBarItemMask & 0x1)
						showTips("隐藏全部描线",65,130)
					else						
						showTips("显示全部描线",65,130)
				}
				onExited: {
					//console.log("button allsee exit")
					//image_allsee.source = "res/icon_allsee_n.png"
					//allsee.color = "#2f3033"					
					hideTips()
				}
				/*
				onReleased: {
					console.log("button allsee release")
					allsee.color = "#2f3033"
				}
				onPressed: {
					console.log("button allsee press")
					allsee.color = "black"
				}*/
				onClicked: {
					console.log("button allsee click")
					EventWrapper.showAllData()
				}
			}	
		}
		
		Rectangle{
			id: restoration
			width: 48
			height: 48
			anchors.top: parent.top
			anchors.topMargin: 49
			anchors.left: parent.left
			anchors.leftMargin: 0
			color: "#2f3033"
			Image {
				id: image_restoration
				anchors.fill:parent;
				source: "res/icon_restoration_n.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					console.log("button restoration enter")
					image_restoration.source = "res/icon_restoration_s.png"
					restoration.color = "#202020"
					
					showTips("相机重置",65,180)
				}
				onExited: {
					console.log("button restoration exit")
					image_restoration.source = "res/icon_restoration_n.png"
					restoration.color = "#2f3033"
					
					hideTips()
				}
				onReleased: {
					console.log("button restoration release")
					restoration.color = "#2f3033"
				}
				onPressed: {
					console.log("button restoration press")
					restoration.color = "black"
				}
				onClicked: {
					console.log("button restoration click")
					EventWrapper.resetCamera()
				}
			}	
		}
		
		Rectangle{
			id: featurepoints
			width: 48
			height: 48
			anchors.top: parent.top
			anchors.topMargin: 98
			anchors.left: parent.left
			anchors.leftMargin: 0
			color: "#2f3033"
			
			Image {
				id: image_featurepoints	
				width: 20
				height:20
				anchors.left: parent.left
				anchors.leftMargin: 14
				anchors.top: parent.top
				anchors.topMargin: 14
				//anchors.fill:parent;
				source: "res/icon_Featurepoints_n.png"
			}			
			MouseArea{
				anchors.fill: parent
				hoverEnabled: true
				onEntered: {
					//console.log("button featurepoints enter")
					//image_featurepoints.source = "res/icon_Featurepoints_s.png"
					//featurepoints.color = "#202020"	
					if( fp_clicked )
						showTips("隐藏特征点",65,230)
					else						
						showTips("显示特征点",65,230)					
				}
				onExited: {
					//console.log("button featurepoints exit")
					//image_featurepoints.source = "res/icon_Featurepoints_n.png"
					//featurepoints.color = "#2f3033"					
					hideTips()
				}
				/*
				onReleased: {
					console.log("button featurepoints release")
					featurepoints.color = "#2f3033"
				}
				onPressed: {
					console.log("button featurepoints press")
					featurepoints.color = "black"
				}*/
				onClicked: {
					if( !fp_clicked )
					{
						fp_clicked = true
						featurepoints.color = "#202020"
						image_featurepoints.source = "res/icon_Featurepoints_s.png"						
						EventWrapper.editFeaturePoints()
						console.log("click featurepoints")
					}else {
						fp_clicked = false
						featurepoints.color = "#2f3033"
						image_featurepoints.source = "res/icon_Featurepoints_n.png"
						EventWrapper.exitFeaturePoints()
						console.log("cancel featurepoints")
					}
				}
			}	
		}
	}
	
	AirBtn{
		id: airBtn2
		objectName: "airbtn2"
		x: root.x+root.width-66
		y: ogrewindow.y+22
		width: 48
		height: 48
		color: "#2f3033"
		flags: Qt.FramelessWindowHint
		visible: false
		Image {			
			id: image_unfold
			width: 24
			height: 24
			anchors.left: parent.left
			anchors.leftMargin: 12
			anchors.top: parent.top
			anchors.topMargin: 12
			source: "res/icon_unfold_n.png"
		}			
		MouseArea{
			anchors.fill: parent
			hoverEnabled: true
			onEntered: {
				console.log("button unfold enter")
				image_unfold.source = "res/icon_unfold_s.png"
				airBtn2.color = "#202020"
			}
			onExited: {
				console.log("button unfold exit")
				image_unfold.source = "res/icon_unfold_n.png"
				airBtn2.color = "#2f3033"
			}
			onReleased: {
				console.log("button unfold release")
				airBtn2.color = "#2f3033"
			}
			onPressed: {
				console.log("button unfold press")
				airBtn2.color = "black"
			}
			onClicked: {
				console.log("button unfold click")
				ogrewindow.setWidth(mainWindow.width - 337);
				//ogrewindow.width = mainWindow.width - 337;	
				//ogrewindow.maximumWidth = Screen.desktopAvailableWidth - 337
				airBtn2.visible = false
				airBtn2.hide()
			}
		}	
	}

	//profile window
	AirBtn{
		id: profileWin
		objectName: "profilewin"
		x: ogrewindow.x
		y: ogrewindow.y+ogrewindow.height-300
		width: 200
		height: 300
		color: "#2f3033"
		flags: Qt.FramelessWindowHint
		visible: false		
		
		FocusScope {
			id: checkbox
			Accessible.role: Accessible.CheckBox

			property string text: "CheckBox"
			property bool checked // required variable
			
			width: 100
			height: 30
			Row {
				spacing: 2
				Rectangle {
					width: 12
					height: 12
					border.width: checkbox.focus ? 2 : 1
					border.color: "black"
					Text {
						id: checkboxText
						text: checkbox.checked ? "x" : ""
						anchors.centerIn: parent
					}
				}
				Text {
					text: checkbox.text
				}
			}
		
			MouseArea {
				anchors.fill: parent
				onClicked: 
				{
					checkbox.checked = !checkbox.checked
					//todo
				}
			}
		}
		
	}
	
	//save window
	AirBtn{
		id: saveWin
		objectName: "savewin"
		//x: mainWindow.x + ogrewindow.width/2 - 190
		//y: mainWindow.y + mainWindow.height/2 - 80
		width: 380
		height: 173
		color: "#2f3033"
		flags: Qt.FramelessWindowHint
		visible: false
		modality: Qt.ApplicationModal

		SaveWindow {
			id:	saveWindow
			
			anchors.fill: parent
			color: "#2f3033"
		}
	}
	
	//delete window
	AirBtn{
		id: deleteWin
		objectName: "deleteWin"
		width: 380
		height: 173
		color: "#2f3033"
		flags: Qt.FramelessWindowHint
		visible: false
		modality: Qt.ApplicationModal

		DeleteWindow {
			anchors.fill: parent
			color: "#2f3033"
		}
	}
	
	//tip window
	AirBtn{
		id: tipWin
		objectName: "tipwin"
		width: 48
		height: 20
		flags: Qt.FramelessWindowHint
		visible: false
		Rectangle{
			anchors.fill: parent
			color: "#E2DDB1"
			border.color: "#000000"
			border.width: 1
			radius: 2
		}
		Text{
			id: tip_text
			text: "AoJ"
			anchors.top: parent.top
			anchors.topMargin: 2
			anchors.left: parent.left
			anchors.leftMargin: 5
			font.pixelSize: 10
			color:"#000000";
			font.family:"微软雅黑"
		}
	}
	
	// msgWin
	AirBtn{
		id: msgWin
		objectName: "msgWin"
		width: 200
		height: 100
		flags: Qt.FramelessWindowHint
		visible: false
		modality: Qt.ApplicationModal
		
		Rectangle{
			anchors.fill: parent
			color: "#2f3033"
		}
		
		Text{
			id: msg_text
			text: ""	
			width: parent.width
			height: 32
			anchors.top: parent.top
			anchors.topMargin: 18
			anchors.left: parent.left
			anchors.leftMargin: 20
			font.pixelSize: 15
			color:"#FFFFFF";
			font.family:"微软雅黑"
		}
		
		Button{
			id: msg_btn
			width: 80
			height: 32
			anchors.left: parent.left
			anchors.leftMargin: 60
			anchors.top: parent.top
			anchors.topMargin: 60
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
				msgWin.hide()
				msgWin.visible = false
			}
		}
	}
	
	Connections {
		target: EventWrapper
		onLoadSuccess: {
			console.log("loadSuccess");
			g_hasModel = true
			enableToolBar()
			showDataWindow()

			if(DataModel.dataList.length > 0)
			{
				DataModel.setCurrDataByUI(DataModel.dataList[0])
				g_currIndex = -1
			}
			
			var itemCount = DataModel.dataList.length + DataModel.newDataList.length
			if ( itemCount == 0 )
			{
				btn_export.enabled = false
				image_export.source = "res/icon_export_false.png"
			}
			

		}
		
		onEstimateDataSuccess:{
			console.log("onEstimateDataSuccess");
			saveCurrData()
			DataModel.currData.setSaved(true)
			setCheckToolBar(1)
			setCheckItem(0)
			btn_saveoredit_text.text = "编辑"
			btn_cancelordelete_text.text = "移除"
		}
		
		onUpdateValue:{
			var value = val*100.0
			data_val.text = toDecimal2(value)
			for (var i = 0; i < DataModel.newDataList.length; i++)
			{
				if (DataModel.newDataList[i] == DataModel.currData)
				{
					break
				}			
			}

			for (var i = 0; i < DataModel.dataList.length; i++)
			{
				if (DataModel.dataList[i] == DataModel.currData)
				{
					var item = listView_data.itemAt(57, 16 + 32 * i)
					var value2 = toDecimal2(value);
					if (value2 != null && item != null)
						item.updateValue(value2)
					break
				}			
			}					
		}
		
		onUpdateEditTime:{
			//DataModel.currData.setEditTime(g_nowTime)
			data_time.text = "上次编辑：" + DataModel.currData.editTime
		}
		
		onSelectedDOM:{
			for (var i = 0; i < DataModel.newDataList.length; i++)
			{
				if (DataModel.newDataList[i].name == name)
				{
					DataModel.currData = DataModel.newDataList[i]
					break
				}			
			}

			for (var i = 0; i < DataModel.dataList.length; i++)
			{
				if (DataModel.dataList[i].name == name)
				{
					DataModel.currData = DataModel.dataList[i]
					break
				}			
			}				
		}
		
		onUpdateToolBarState: {
			setCheckToolBar(num)
		}
		
		onUpdateFloatBarState: {
			setCheckFloatBar(num)
		}
		
		onUpdateFeaturepointsState: {
			if(num == 0)
			{
				fp_clicked = false
				featurepoints.color = "#2f3033"
				image_featurepoints.source = "res/icon_Featurepoints_n.png"			
			}else{
				fp_clicked = true
				featurepoints.color = "#202020"
				image_featurepoints.source = "res/icon_Featurepoints_s.png"						
			}
		}
		
		onDirtyChanged: {
			console.log("onDirtyChanged:")
			console.log(isDirty)
			if( isDirty )
			{
				btn_saveoredit.enabled = true
				btn_saveoredit.color = "#202020"
				btn_saveoredit_text.text = "确定"
				btn_export.enabled = false
				image_export.source = "res/icon_export_false.png"
			}else{				
				btn_saveoredit_text.text = "编辑"
				btn_export.enabled = true
				image_export.source = "res/icon_export_n.png"
				
				if(g_clickItemNum != 0)
				{
					if (g_clickItemNum > 0)
					{
						DataModel.setCurrDataByUI(DataModel.newDataList[g_clickItemNum-1])
					}else{
						DataModel.setCurrDataByUI(DataModel.dataList[-g_clickItemNum-1])
					}
					g_clickItemNum = 0
					g_bmsg = false
				}
				
				if(g_bmsg)
				{
					console.log(g_createItem)
					if(g_createItem >= 0)
					{
						EventWrapper.createItem(g_createItem)
						btn_cancelordelete_text.text = "移除"
						btn_saveoredit.enabled = false
						btn_saveoredit.color = "#2A2B2E"						
						g_createItem = -1
					}else{	
						console.log("g_currIndex:" +g_currIndex)
						if (g_currIndex > 0)
						{
							DataModel.setCurrDataByUI(DataModel.newDataList[g_currIndex-1])
						}else{
							DataModel.setCurrDataByUI(DataModel.dataList[-g_currIndex-1])
						}
					}
					
					switch(g_next_msg)
					{
					case 1:
					{
						g_next_msg = 0
						if ( DataModel.isDirty )
						{
							saveWin.show()
							saveWin.visible = true
							saveWindow.setText("打开新文档前 是否 保存当前文档的更改？", 1)
						}else{
							if (EventWrapper.openFileDlg())
							{
								disableToolBar()
							}
						}
					}
					break
					case 2:
					{
						g_next_msg = 0
						saveFile()
					}
					break
					case 3:
					{
						g_next_msg = 0
						EventWrapper.saveasAllData()
					}
					break
					case 4:
					{
						if ( DataModel.isDirty )
						{
							saveWin.show()
							saveWin.visible = true
							saveWindow.setText("关闭前 是否 保存当前文档的更改？", 2)
						}else{
							mainWindow.close()
						}
					}
					break
					case 5:
					{
						g_next_msg = 0
						EventWrapper.exportData()
					}
					break
					default:
					break
					}	
					
					g_bmsg = false
				}
			}
		}
		
		onSaveAlldataOver: {
			console.log("onSaveAlldataOver")
			console.log(g_next_openorclose)
			if(g_next_openorclose == 0)
			{
				if (EventWrapper.openFileDlg())
				{
					root.disableToolBar()
				}
			}else if(g_next_openorclose == 1)
			{
				mainWindow.close()
			}
			g_next_openorclose = -1
		}
	}
	
	Connections {
		target: DataModel
		onCurrDataChanged: {
			console.log("onCurrDataChanged")
			//if(DataModel.currData == null)
			//	return
			
			data_val.text = toDecimal2(DataModel.currData.value * 100.0)
			//data_name.text = DataModel.currData.name
			data_name_item.updateDataName(DataModel.currData.name)
			data_type.text = "类型：" + DataModel.currData.type
			data_time.text = "上次编辑：" + DataModel.currData.editTime
			//btn_edit.visible = DataModel.currData.neworOld
			
			//console.log("保存："+DataModel.currData.isDirty)
			if (DataModel.currData.isDirty)
			{	//保存
				btn_saveoredit_text.text = "确定"
				btn_cancelordelete_text.text = "取消"
			}else{	//编辑
				btn_saveoredit_text.text = "编辑"
				btn_cancelordelete_text.text = "移除"
			}
			
			var index = 0;
			if (DataModel.currData.neworOld)
			{
				for (var i = 0; i < DataModel.newDataList.length; i++)
				{
					if (DataModel.newDataList[i] == DataModel.currData)
						break;
					index++;				
				}
				//通知组件list_newdata_delegate选中index
				setCheckNewItem(index)				
			}else{				
				for (var i = 0; i < DataModel.dataList.length; i++)
				{
					if (DataModel.dataList[i] == DataModel.currData)
						break;
					index++;				
				}
				//通知组件list_data_delegate选中index
				setCheckItem(index)
			}
			
			var num = 0x1
			if (DataModel.currData.type == "高度")
			{
				num |= 0x2
			}else if (DataModel.currData.type == "长度")
			{
				num |= 0x4				
			}else if (DataModel.currData.type == "表面长度")
			{
				num |= 0x8
			}else if (DataModel.currData.type == "表面围度")
			{
				num |= 0x10
			}else if (DataModel.currData.type == "围度")
			{
				num |= 0x20
			}
			setCheckToolBar(num)
			
			console.log("onCurrDataChanged over")
		}
		
		onClickCanceled: {
			btn_saveoredit_text.text = "编辑"
			btn_cancelordelete_text.text = "移除"
		}
	}
}
