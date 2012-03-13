# http://aumhaa.blogspot.com

import Live
import time
import math

""" _Framework files """
from _Framework.ButtonElement import ButtonElement # Class representing a button a the controller
from _Framework.ButtonMatrixElement import ButtonMatrixElement # Class representing a 2-dimensional set of buttons
#from _Framework.ButtonSliderElement import ButtonSliderElement # Class representing a set of buttons used as a slider
from _Framework.ChannelStripComponent import ChannelStripComponent # Class attaching to the mixer of a given track
#from _Framework.ChannelTranslationSelector import ChannelTranslationSelector # Class switches modes by translating the given controls' message channel
from _Framework.ClipSlotComponent import ClipSlotComponent # Class representing a ClipSlot within Live
from _Framework.CompoundComponent import CompoundComponent # Base class for classes encompasing other components to form complex components
from _Framework.ControlElement import ControlElement # Base class for all classes representing control elements on a controller 
from _Framework.ControlSurface import ControlSurface # Central base class for scripts based on the new Framework
from _Framework.ControlSurfaceComponent import ControlSurfaceComponent # Base class for all classes encapsulating functions in Live
from _Framework.DeviceComponent import DeviceComponent # Class representing a device in Live
#from _Framework.DisplayDataSource import DisplayDataSource # Data object that is fed with a specific string and notifies its observers
from _Framework.EncoderElement import EncoderElement # Class representing a continuous control on the controller
from _Framework.InputControlElement import * # Base class for all classes representing control elements on a controller
#from _Framework.LogicalDisplaySegment import LogicalDisplaySegment # Class representing a specific segment of a display on the controller
from _Framework.MixerComponent import MixerComponent # Class encompassing several channel strips to form a mixer
from _Framework.ModeSelectorComponent import ModeSelectorComponent # Class for switching between modes, handle several functions with few controls
from _Framework.NotifyingControlElement import NotifyingControlElement # Class representing control elements that can send values
#from _Framework.PhysicalDisplayElement import PhysicalDisplayElement # Class representing a display on the controller
from _Framework.SceneComponent import SceneComponent # Class representing a scene in Live
from _Framework.SessionComponent import SessionComponent # Class encompassing several scene to cover a defined section of Live's session
from _Framework.SessionZoomingComponent import SessionZoomingComponent # Class using a matrix of buttons to choose blocks of clips in the session
from _Framework.SliderElement import SliderElement # Class representing a slider on the controller
from _Framework.TrackEQComponent import TrackEQComponent # Class representing a track's EQ, it attaches to the last EQ device in the track
from _Framework.TrackFilterComponent import TrackFilterComponent # Class representing a track's filter, attaches to the last filter in the track
from _Framework.TransportComponent import TransportComponent # Class encapsulating all functions in Live's transport section

"""Custom files, overrides, and files from other scripts"""
from ShiftModeComponent import ShiftModeComponent
from FunctionModeComponent import FunctionModeComponent
from FlashingButtonElement import FlashingButtonElement
from MonoEncoderElement2 import MonoEncoderElement2
from DeviceSelectorComponent import DeviceSelectorComponent
from DetailViewControllerComponent import DetailViewControllerComponent
from ResetSendsComponent import ResetSendsComponent
from MonoBridgeElement import MonoBridgeElement
from MonomodComponent import MonomodComponent
from MonomodModeComponent import MonomodModeComponent
from SwitchboardElement import SwitchboardElement
from MonoClient import MonoClient
from CodecEncoderElement import CodecEncoderElement
from EncoderMatrixElement import EncoderMatrixElement



import LiveUtils
from _Generic.Devices import *
from Cntrlr_Map import *

""" Here we define some global variables """
CHANNEL = 0 
session = None 
mixer = None 

switchxfader = (240, 00, 01, 97, 02, 15, 01, 247)
switchxfaderrgb = (240, 00, 01, 97, 07, 15, 01, 247)
assigncolors = (240, 00, 01, 97, 07, 34, 00, 07, 03, 06, 05, 01, 02, 04, 247)
assign_default_colors = (240, 00, 01, 97, 07, 34, 00, 07, 06, 05, 01, 04, 03, 02, 247)
check_model = (240, 126, 127, 6, 1, 247)


factoryreset = (240,0,1,97,4,6,247)
btn_channels = (240, 0, 1, 97, 8, 19, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, 0, 247);
enc_channels = (240, 0, 1, 97, 8, 20, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, CHANNEL, 247);
SLOWENCODER = (240, 0, 1, 97, 8, 30, 69, 00, 247)
NORMALENCODER = (240, 0, 1, 97, 8, 30, 00, 00, 247)
FASTENCODER = (240, 0, 1, 97, 8, 30, 04, 00, 247)

class Cntrlr(ControlSurface):
	__module__ = __name__
	__doc__ = " MonOhmod companion controller script "


	def __init__(self, c_instance):
		"""everything except the '_on_selected_track_changed' override and 'disconnect' runs from here"""
		ControlSurface.__init__(self, c_instance)
		self.set_suppress_rebuild_requests(True)
		self._version_check = 'b993'
		self._host_name = 'Cntrlr'
		self._color_type = 'OhmRGB'
		self.log_message("--------------= CNTRLR log opened =--------------") 
		self._hosts = []
		self.hosts = []
		self._client = [None for index in range(4)]
		self._active_client = None
		self._bright = True
		self._rgb = 0 ##will change which color scheme is used, 0 is Livid 1 is AumHaa 2 is Monochrome(deprecated)
		self._timer = 0		
		self._touched = 0
		self._local_ring_control = True
		self.set_local_ring_control(1)
		self._absolute_mode = 1
		self.flash_status = 1
		self._is_split = True
		self._backlight = 127
		self._backlight_type = 'static'
		self._ohm = 127
		self._ohm_type = 'static'
		#self._pad_translations = PAD_TRANSLATION
		self._setup_monobridge()
		self._setup_controls()
		self._setup_transport_control() 
		self._setup_mixer_control()
		self._setup_session_control()
		self._assign_session_colors()
		self._setup_device_control()
		self._setup_device_selector()
		self._setup_minusmod()
		self._setup_switchboard()
		self._setup_modes() 
		self.set_suppress_rebuild_requests(False)
		self.song().view.add_selected_track_listener(self._update_selected_device)
		self.show_message('Cntrl Control Surface Loaded')
		#if FORCE_TYPE is True:
		#	self._rgb = FORCE_COLOR_TYPE
		#else:
		#	self.schedule_message(10, self.query_ohm, None)
	

	"""script initialization methods"""
	def _setup_monobridge(self):
		self._monobridge = MonoBridgeElement(self)
		self._monobridge.name = 'MonoBridge'
	

	def _setup_controls(self):
		is_momentary = True
		self._fader = [None for index in range(8)]
		self._dial_left = [None for index in range(12)]
		self._dial_right = [None for index in range(12)]
		self._encoder = [None for index in range(12)]	
		self._encoder_button = [None for index in range(12)]	
		self._grid = [None for index in range(16)]
		self._button = [None for index in range(32)]
		for index in range(8):
			self._fader[index] = MonoEncoderElement2(MIDI_CC_TYPE, CHANNEL, CNTRLR_FADERS[index], Live.MidiMap.MapMode.absolute, 'Fader_' + str(index), index, self)
		self._knobs = []
		for index in range(12):
			self._dial_left[index] = MonoEncoderElement2(MIDI_CC_TYPE, CHANNEL, CNTRLR_KNOBS_LEFT[index], Live.MidiMap.MapMode.absolute, 'Dial_Left_' + str(index), CNTRLR_KNOBS_LEFT[index], self)
			self._knobs.append(self._dial_left[index])
		for index in range(12):
			self._dial_right[index] = MonoEncoderElement2(MIDI_CC_TYPE, CHANNEL, CNTRLR_KNOBS_RIGHT[index], Live.MidiMap.MapMode.absolute, 'Dial_Right_' + str(index), CNTRLR_KNOBS_RIGHT[index], self)
			self._knobs.append(self._dial_right[index])
		for index in range(12):
			#self._encoder[index] = MonoEncoderElement2(MIDI_CC_TYPE, CHANNEL, CNTRLR_DIALS[index], Live.MidiMap.MapMode.absolute, 'Encoder_' + str(index), CNTRLR_DIALS[index], self)
			self._encoder[index] = CodecEncoderElement(MIDI_CC_TYPE, CHANNEL, CNTRLR_DIALS[index], Live.MidiMap.MapMode.absolute, 'Encoder_' + str(index), CNTRLR_DIALS[index], self)
		for index in range(12):
			self._encoder_button[index] = FlashingButtonElement(is_momentary, MIDI_NOTE_TYPE, CHANNEL, CNTRLR_DIAL_BUTTONS[index], 'Encoder_Button_' + str(index), self)
		for index in range(16):
			self._grid[index] = FlashingButtonElement(is_momentary, MIDI_NOTE_TYPE, CHANNEL, CNTRLR_GRID[index], 'Grid' + str(index), self)
		for index in range(32):
			self._button[index] = FlashingButtonElement(is_momentary, MIDI_NOTE_TYPE, CHANNEL, CNTRLR_BUTTONS[index], 'Button_' + str(index), self)
		self._matrix = ButtonMatrixElement()
		self._matrix.name = 'Matrix'
		self._dial_matrix = EncoderMatrixElement(self)
		self._dial_matrix.name = 'Dial_Matrix'
		self._dial_button_matrix = ButtonMatrixElement()
		self._dial_button_matrix.name = 'Dial_Button_Matrix'

		for row in range(4):
			button_row = []
			for column in range(4):
				button_row.append(self._grid[(row*4) + column])
			self._matrix.add_row(tuple(button_row)) 
		for row in range(3):
			dial_row = []
			for column in range(4):
				dial_row.append(self._encoder[(row*4) + column])
			self._dial_matrix.add_row(tuple(dial_row))
		for row in range(3):
			dial_button_row = []
			for column in range(4):
				dial_button_row.append(self._encoder_button[(row*4) + column])
			self._dial_button_matrix.add_row(tuple(dial_button_row))
	

	def _setup_transport_control(self):
		self._transport = TransportComponent() 
		self._transport.name = 'Transport'
	

	def _setup_mixer_control(self):
		is_momentary = True
		self._num_tracks = (4) #A mixer is one-dimensional; 
		self._mixer = MixerComponent(4, 2, True, False)
		self._mixer.name = 'Mixer'
		self._mixer.set_track_offset(0) #Sets start point for mixer strip (offset from left)
		for index in range(4):
			self._mixer.channel_strip(index).set_volume_control(self._fader[index])
		for index in range(4):
			self._mixer.channel_strip(index).name = 'Mixer_ChannelStrip_' + str(index)
			self._mixer.track_eq(index).name = 'Mixer_EQ_' + str(index)
			self._mixer.channel_strip(index)._invert_mute_feedback = True
		self.song().view.selected_track = self._mixer.channel_strip(0)._track #set the selected strip to the first track, so that we don't, for example, try to assign a button to arm the master track, which would cause an assertion error
		self._send_reset = ResetSendsComponent(self)
		self._send_reset.name = 'Sends_Reset'
	

	def _setup_session_control(self):
		is_momentary = True
		num_tracks = 4
		num_scenes = 4 
		self._session = SessionComponent(num_tracks, num_scenes)
		self._session.name = "Session"
		self._session.set_offsets(0, 0)
		self._session.set_stop_track_clip_value(STOP_CLIP[self._rgb])
		self._scene = [None for index in range(4)]
		for row in range(num_scenes):
			self._scene[row] = self._session.scene(row)
			self._scene[row].name = 'Scene_' + str(row)
			for column in range(num_tracks):
				clip_slot = self._scene[row].clip_slot(column)
				clip_slot.name = str(column) + '_Clip_Slot' + str(row)
				clip_slot.set_triggered_to_play_value(CLIP_TRG_PLAY[self._rgb])
				clip_slot.set_triggered_to_record_value(CLIP_TRG_REC[self._rgb])
				clip_slot.set_stopped_value(CLIP_STOP[self._rgb])
				clip_slot.set_started_value(CLIP_STARTED[self._rgb])
				clip_slot.set_recording_value(CLIP_RECORDING[self._rgb])
		self._session.set_mixer(self._mixer)
		self._session_zoom = SessionZoomingComponent(self._session)	 
		self._session_zoom.name = 'Session_Overview'
		self._session_zoom.set_button_matrix(self._matrix)
		self._session_zoom.set_zoom_button(self._button[31])
		#self._session_zoom.set_enabled(True) 
	

	def _assign_session_colors(self):
		num_tracks = 4
		num_scenes = 4 
		self._session.set_stop_track_clip_value(STOP_ALL[self._rgb])
		for row in range(num_scenes): 
			for column in range(num_tracks):
				self._scene[row].clip_slot(column).set_triggered_to_play_value(CLIP_TRG_PLAY[self._rgb])
				self._scene[row].clip_slot(column).set_triggered_to_record_value(CLIP_TRG_REC[self._rgb])
				self._scene[row].clip_slot(column).set_stopped_value(CLIP_STOP[self._rgb])
				self._scene[row].clip_slot(column).set_started_value(CLIP_STARTED[self._rgb])
				self._scene[row].clip_slot(column).set_recording_value(CLIP_RECORDING[self._rgb])		
		self._session_zoom.set_stopped_value(ZOOM_STOPPED[self._rgb])
		self._session_zoom.set_playing_value(ZOOM_PLAYING[self._rgb])
		self._session_zoom.set_selected_value(ZOOM_SELECTED[self._rgb])
		self.refresh_state()
	

	def _setup_device_control(self):
		self._device = DeviceComponent()
		self._device.name = 'Device_Component'
		#self._device._update = self._device_update(self._device)
		self._device.set_parameter_controls(tuple([self._encoder[index+4] for index in range(8)]))
		self.set_device_component(self._device)
		self._device_navigator = DetailViewControllerComponent(self)
		self._device_navigator.name = 'Device_Navigator'
		self._device_selection_follows_track_selection = FOLLOW
	

	def _setup_device_selector(self):
		self._device_selector = DeviceSelectorComponent(self)
		self._device_selector.name = 'Device_Selector'
	

	def _setup_minusmod(self):
		self._host = MonomodComponent(self)
		self._host.name = 'Monomod_Host'
		self.hosts = [self._host]
		self._hosts = [self._host]
		for index in range(4):
			self._client[index] = MonoClient(self, index)
			self._client[index].name = 'Client_' + str(index)
			self._client[index]._mod_dial = self._encoder[index]
		self._active_client = self._client[0]
		self._active_client._is_active = True
		self._host.connect_to_clients(self)
	

	def _setup_switchboard(self):
		self._switchboard = SwitchboardElement(self, self._client)
		self._switchboard.name = 'Switchboard'
	

	def _setup_modes(self):
		self._shift_mode = ShiftModeComponent(self, self.shift_update) 
		self._shift_mode.name = 'Mod_Mode'
		self._shift_mode.set_mode_buttons([self._encoder_button[index] for index in range(4)])
	

	"""cntrlr modes"""

	def deassign_live_controls(self):
		for index in range(4):
			if self._encoder[index].value_has_listener(self._client[index]._mod_dial_value):
				self._encoder[index].remove_value_listener(self._client[index]._mod_dial_value)
		for index in range(4):
			self._mixer.channel_strip(index).set_solo_button(None)
			self._mixer.channel_strip(index).set_arm_button(None)
			self._mixer.channel_strip(index).set_mute_button(None)
			self._mixer.channel_strip(index).set_select_button(None)
		#for index in range(2):
		#	self._mixer.return_strip(index).set_volume_control(self._fader[index+4])
		#self._mixer.master_strip().set_volume_control(self._fader[7])
		#self._mixer.set_prehear_volume_control(self._fader[6])	
		for column in range(4):
			for row in range(4):
				self._scene[row].clip_slot(column).set_launch_button(None)
		self._send_reset.set_buttons(tuple([None for index in range(4)]))
		self._session.set_stop_track_clip_buttons(None)
		self._transport.set_play_button(None)	
		self._transport.set_record_button(None)
		self._transport.set_stop_button(None)
		for index in range(16):
			self._grid[index].set_on_off_values(127, 0)
			self._grid[index].reset()
		for index in range(32):
			self._button[index].set_on_off_values(127, 0)
			self._button[index].reset()				
#		for track in range(4):
#			self._mixer.channel_strip(track).set_send_controls(None)
#			self._mixer.channel_strip(track).set_pan_control(None)
#			self._mixer.track_eq(track).set_enabled(False)
		self._device.set_parameter_controls(tuple([self._encoder[index+4] for index in range(8)]))
		self._device_navigator.set_device_nav_buttons(None, None) 
		self._device_navigator.set_enabled(False)
		self._device.set_on_off_button(None)
		self._device.set_lock_button(None)
		self._device.set_bank_nav_buttons(None, None) 
		self._device.set_enabled(False)
		self._session.set_enabled(False)
		self._session_zoom.set_enabled(False)
		for index in range(16):
			self._grid[index].clear_send_cache()
		for index in range(32):
			self._button[index].clear_send_cache()
		for index in range(12):
			self._encoder[index].send_value(0, True)
			self._encoder[index].clear_send_cache()
		for index in range(8):
			self._encoder_button[index+4].send_value(0, True)
			self._encoder_button[index+4].clear_send_cache()
	

	def assign_live_controls(self):
		for index in range(16):
			self._grid[index].clear_send_cache()
		for index in range(32):
			self._button[index].clear_send_cache()
		for index in range(8):
			self._encoder_button[index+4].send_value(0, True)
			self._encoder_button[index+4].clear_send_cache()
		for index in range(12):
			self._encoder[index].send_value(0, True)
			self._encoder[index].clear_send_cache()


		for index in range(4):
			if not self._encoder[index].value_has_listener(self._client[index]._mod_dial_value):
				self._encoder[index].add_value_listener(self._client[index]._mod_dial_value)
			self._client[index].receive_mod_vol(self._client[index]._mod_vol)
		for index in range(4):
			self._button[index].set_on_value(SOLO[self._rgb])
			self._mixer.channel_strip(index).set_solo_button(self._button[index])
			self._button[index+4].set_on_value(ARM[self._rgb])
			self._mixer.channel_strip(index).set_arm_button(self._button[index+4])
			self._button[index+16].set_on_value(MUTE[self._rgb])
			self._mixer.channel_strip(index).set_mute_button(self._button[index+16])
			self._button[index+20].set_on_value(SELECT[self._rgb])
			self._mixer.channel_strip(index).set_select_button(self._button[index+20])
		for index in range(2):
			self._mixer.return_strip(index).set_volume_control(self._fader[index+4])
		self._mixer.master_strip().set_volume_control(self._fader[7])
		self._mixer.set_prehear_volume_control(self._fader[6])
		for track in range(4):
			channel_strip_send_controls = []
			for control in range(2):
				channel_strip_send_controls.append(self._dial_left[track + (control * 4)])
			self._mixer.channel_strip(track).set_send_controls(tuple(channel_strip_send_controls))
			self._mixer.channel_strip(track).set_pan_control(self._dial_left[track + 8])
			gain_controls = []
			self._mixer.track_eq(track).set_gain_controls(tuple([self._dial_right[track+8], self._dial_right[track+4], self._dial_right[track]]))
			self._mixer.track_eq(track).set_enabled(True)	
		for column in range(4):
			for row in range(4):
				self._scene[row].clip_slot(column).set_launch_button(self._grid[(row*4)+column])
		self._send_reset.set_buttons(tuple(self._button[index + 8] for index in range(4)))
		self._session.set_stop_track_clip_buttons(tuple(self._button[index+24] for index in range(4)))
		for index in range(4):
			self._button[index+8].send_value(SEND_RESET[self._rgb], True)
			self._button[index + 24].set_on_off_values(STOP_CLIP[self._rgb], STOP_CLIP[self._rgb])
			self._button[index+24].send_value(STOP_CLIP[self._rgb], True)
		self._button[28].set_on_off_values(PLAY_ON[self._rgb], PLAY[self._rgb])
		self._transport.set_play_button(self._button[28])	
		self._button[30].set_on_off_values(RECORD_ON[self._rgb], RECORD[self._rgb])
		self._transport.set_record_button(self._button[30])
		self._button[29].set_on_value(STOP[self._rgb])
		self._transport.set_stop_button(self._button[29])
		self._button[29].send_value(STOP_OFF[self._rgb], True)
		#self._device.set_parameter_controls(tuple([self._encoder[index+4] for index in range(8)]))
		for index in range(4):
			self._button[index + 12].set_on_off_values(SESSION_NAV[self._rgb], SESSION_NAV_OFF[self._rgb])
		self._session.set_track_bank_buttons(self._button[15], self._button[14])
		self._session.set_scene_bank_buttons(self._button[13], self._button[12])
		self._encoder_button[7].set_on_value(DEVICE_LOCK[self._rgb])
		self._device.set_lock_button(self._encoder_button[7])
		self._encoder_button[4].set_on_value(DEVICE_ON[self._rgb])
		self._device.set_on_off_button(self._encoder_button[4])
		for index in range(2):
			self._encoder_button[index + 8].set_on_value(DEVICE_NAV[self._rgb])
			self._encoder_button[index + 10].set_on_value(DEVICE_BANK[self._rgb])
		self._device_navigator.set_device_nav_buttons(self._encoder_button[10], self._encoder_button[11]) 
		self._device.set_bank_nav_buttons(self._encoder_button[8], self._encoder_button[9]) 
		self._device.set_enabled(True)
		self._device_navigator.set_enabled(True)
		self._session.set_enabled(True)
		self._session_zoom.set_enabled(True)
		self._device.update()
		self._session.update()
	

	"""function mode callbacks"""

	def shift_update(self):
		self.assign_alternate_mappings(0)
		for index in range(4):
			self._shift_mode._modes_buttons[index].send_value(self._client[index]._mod_color)
		if self._shift_mode._mode_index is 0:
			self._host._set_dial_matrix(None, None)
			#self._host._set_knobs(None)
			self._host._set_button_matrix(None)
			self._host._set_key_buttons(None)
			self._host.set_enabled(False)
			self.set_local_ring_control(1)
			self.assign_live_controls()
			#for index in range(4):
			#	self._shift_mode._modes_buttons[index].turn_off()
		else:
			self.deassign_live_controls()
			#self.set_local_ring_control(self._host._active_client._local_ring_control)
			self._host.set_enabled(True)
			self._host._set_dial_matrix(self._dial_matrix, self._dial_button_matrix)
			#self._host._set_knobs(tuple(self._knobs))
			self._host._set_button_matrix(self._matrix)
			self._host._set_key_buttons(tuple(self._button))
			self._host._select_client(self._shift_mode._mode_index-1)
			for index in range(4):
				if self._shift_mode._mode_index == (index + 1):
					self._shift_mode._modes_buttons[index].send_value(1)
			if not self._host._active_client.is_connected():
				self.assign_alternate_mappings(self._shift_mode._mode_index)
				
	"""called on timer"""
	def update_display(self):
		""" Live -> Script
		Aka on_timer. Called every 100 ms and should be used to update display relevant
		parts of the controller
		"""
		for message in self._scheduled_messages:
			message['Delay'] -= 1
			if (message['Delay'] == 0):
				if (message['Parameter'] != None):
					message['Message'](message['Parameter'])
				else:
					message['Message']()
					del self._scheduled_messages[self._scheduled_messages.index(message)]

		for callback in self._timer_callbacks:
			callback()
		self._timer = (self._timer + 1) % 256
		if(self._local_ring_control is False):
			self.send_ring_leds()
		self.flash()
		#self.strobe()
	

	def flash(self):
		#if(self.flash_status > 0):
		for index in range(32):
			if(self._button[index]._flash_state > 0):
				self._button[index].flash(self._timer)
		for index in range(16):
			if(self._grid[index]._flash_state > 0):
				self._grid[index].flash(self._timer)
		for index in range(12):
			if(self._encoder_button[index]._flash_state > 0):
				self._encoder_button[index].flash(self._timer)			
	

	def strobe(self):
		if(self._backlight_type != 'static'):
			if(self._backlight_type is 'pulse'):
				self._backlight = int(math.fabs(((self._timer * 8) % 64) -32) +32)
			if(self._backlight_type is 'up'):
				self._backlight = int(((self._timer * 4) % 64) + 16)
			if(self._backlight_type is 'down'):
				self._backlight = int(math.fabs(int(((self._timer * 4) % 64) - 64)) + 16)
		if(self._rgb == 1):
			self._send_midi(tuple([176, 27, int(self._backlight)]))
		else:
			self._send_midi(tuple([176, 118, int(self._backlight)]))
		if(self._ohm_type != 'static'):
			if(self._ohm_type is 'pulse'):
				self._ohm = int(math.fabs(((self._timer * 8) % 64) -32) +32)
			if(self._ohm_type is 'up'):
				self._ohm = int(((self._timer * 4) % 64) + 16)
			if(self._ohm_type is 'down'):
				self._ohm = int(math.fabs(int(((self._timer * 4) % 64) - 64)) + 16)
		if(self._rgb == 1):
			self._send_midi(tuple([176, 63, int(self._ohm)]))
			self._send_midi(tuple([176, 31, int(self._ohm)]))	
		else:
			self._send_midi(tuple([176, 119, int(self._ohm)]))
	


	"""m4l bridge"""
	def generate_strip_string(self, display_string):
		#try:
		#	display_string = str(display_string)
		#except:
		#	return ' ? '
		#else:
			#self.log_message(display_string)
		NUM_CHARS_PER_DISPLAY_STRIP = 12
		if (not display_string):
			return (' ' * NUM_CHARS_PER_DISPLAY_STRIP)
		if ((len(display_string.strip()) > (NUM_CHARS_PER_DISPLAY_STRIP - 1)) and (display_string.endswith('dB') and (display_string.find('.') != -1))):
			display_string = display_string[:-2]
		if (len(display_string) > (NUM_CHARS_PER_DISPLAY_STRIP - 1)):
			for um in [' ',
			 'i',
			 'o',
			 'u',
			 'e',
			 'a']:
				while ((len(display_string) > (NUM_CHARS_PER_DISPLAY_STRIP - 1)) and (display_string.rfind(um, 1) != -1)):
					um_pos = display_string.rfind(um, 1)
					display_string = (display_string[:um_pos] + display_string[(um_pos + 1):])
		else:
			display_string = display_string.center((NUM_CHARS_PER_DISPLAY_STRIP - 1))
		ret = u''
		for i in range((NUM_CHARS_PER_DISPLAY_STRIP - 1)):
			if ((ord(display_string[i]) > 127) or (ord(display_string[i]) < 0)):
				ret += ' '
			else:
				ret += display_string[i]

		ret += ' '
		assert (len(ret) == NUM_CHARS_PER_DISPLAY_STRIP)
		return ret
	

	def notification_to_bridge(self, name, value, sender):
		if(isinstance(sender, (MonoEncoderElement2, CodecEncoderElement))):
			self._monobridge._send(sender.name, 'lcd_name', str(self.generate_strip_string(name)))
			self._monobridge._send(sender.name, 'lcd_value', str(self.generate_strip_string(value)))
	

	def touched(self):
		if self._touched is 0:
			self._monobridge._send('touch', 'on')
			self.schedule_message(2, self.check_touch)
		self._touched +=1
	

	def check_touch(self):
		if self._touched > 5:
			self._touched = 5
		elif self._touched > 0:
			self._touched -= 1
		if self._touched is 0:
			self._monobridge._send('touch', 'off')
		else:
			self.schedule_message(2, self.check_touch)
	

	def get_clip_names(self):
		clip_names = []
		for scene in self._session._scenes:
			for clip_slot in scene._clip_slots:
				if clip_slot.has_clip() is True:
					clip_names.append(clip_slot._clip_slot)##.clip.name)
					return clip_slot._clip_slot
					##self.log_message(str(clip_slot._clip_slot.clip.name))
		return clip_names
	


	"""midi functionality"""
	def max_to_midi(self, message): #takes a 'tosymbol' list from Max, such as "240 126 0 6 1 247"
		msg_str = str(message) #gets rid of the quotation marks which 'tosymbol' has added
		midi_msg = tuple(int(s) for s in msg_str.split()) #converts to a tuple 
		self._send_midi(midi_msg) #sends to controller
	

	def max_from_midi(self, message): #takes a 'tosymbol' list from Max, such as "240 126 0 6 1 247"
		msg_str = str(message) #gets rid of the quotation marks which 'tosymbol' has added
		midi_msg = tuple(int(s) for s in msg_str.split()) #converts to a tuple 
		self.receive_external_midi(midi_msg) #sends to controller
	

	def to_encoder(self, num, val):
		rv=int(val*127)
		self._device._parameter_controls[num].receive_value(rv)
		p = self._device._parameter_controls[num]._parameter_to_map_to
		newval = (val * (p.max - p.min)) + p.min
		p.value = newval
	

#	def handle_sysex(self, midi_bytes):
#		assert(isinstance (midi_bytes, tuple))
#		#self.log_message(str('sysex') + str(midi_bytes))
#		if len(midi_bytes) > 10:
#			##if midi_bytes == tuple([240, 126, 0, 6, 2, 0, 1, 97, 1, 0, 7, 0, 15, 10, 0, 0, 247]):
#			if midi_bytes[:11] == tuple([240, 126, 0, 6, 2, 0, 1, 97, 1, 0, 7]):
#				self.show_message(str('Ohm64 RGB detected...setting color map'))
#				self.log_message(str('Ohm64 RGB detected...setting color map'))
#				self._rgb = 0
#				self._host._host_name = 'OhmRGB'
#				self._color_type = 'OhmRGB'
#			#elif midi_bytes == tuple([240, 126, 0, 6, 2, 0, 1, 97, 1, 0, 2, 0, 0, 1, 1, 0, 247]):
#			elif midi_bytes[:11] == tuple([240, 126, 0, 6, 2, 0, 1, 97, 1, 0, 2]):
#				self.show_message(str('Ohm64 Monochrome detected...setting color map'))
#				self.log_message(str('Ohm64 Monochrome detected...setting color map'))
#				self._rgb = 1
#				self._host._host_name = 'Ohm64'
#				self._color_type = 'Monochrome'
	

	def receive_external_midi(self, midi_bytes):
		#self.log_message('receive_external_midi' + str(midi_bytes))
		assert (midi_bytes != None)
		assert isinstance(midi_bytes, tuple)
		self.set_suppress_rebuild_requests(True)
		if (len(midi_bytes) is 3):
			msg_type = (midi_bytes[0] & 240)
			forwarding_key = [midi_bytes[0]]
			self.log_message(str(self._forwarding_registry))
			if (msg_type is not MIDI_PB_TYPE):
				forwarding_key.append(midi_bytes[1])
			recipient = self._forwarding_registry[tuple(forwarding_key)]
			self.log_message('receive_midi recipient ' + str(recipient))
			if (recipient != None):
				recipient.receive_value(midi_bytes[2])
		else:
			self.handle_sysex(midi_bytes)
		self.set_suppress_rebuild_requests(False)
	

	def set_local_ring_control(self, val = 1):
		#assert val is isinstance(val, type(False))
		if val != self._local_ring_control:
			self._local_ring_control = (val!=0)
			if(self._local_ring_control is True):
				self._send_midi(tuple([240, 0, 1, 97, 8, 32, 0, 247]))
			else:
				self._send_midi(tuple([240, 0, 1, 97, 8, 32, 1, 247]))
	

	def send_ring_leds(self):
		leds = [240, 0, 1, 97, 8, 31]
		for index in range(12):
				wheel = self._encoder[index]
				bytes = wheel._get_ring()
				leds.append(bytes[0])
				leds.append(int(bytes[1]) + int(bytes[2]))
				#if(row == 1 and column == 0):
				#	self.log_message(str(leds) + ' ' + str(bytes[0]) + ' ' + str(bytes[1]) + ' ' + str(bytes[2]))
		leds.append(247)
		self._send_midi(tuple(leds))
	

	def set_absolute_mode(self, val = 1):
		self._absolute_mode = (val!=0)
		if self._absolute_mode is True:
			self._send_midi(tuple([240, 0, 1, 97, 4, 17, 0, 0, 0, 0, 0, 0, 0, 0, 247]))
		else:
			self._send_midi(tuple([240, 0, 1, 97, 4, 17, 127, 127, 127, 127, 127, 127, 127, 127, 247]))
	

	"""general functionality"""
	def disconnect(self):
		"""clean things up on disconnect"""
		self.deassign_live_controls()
		self.song().view.remove_selected_track_listener(self._update_selected_device)
		self._hosts = []
		#self._disconnect_notifier.set_mode(0)
		self.log_message("--------------= CNTRLR log closed =--------------") #Create entry in log file
		ControlSurface.disconnect(self)
		return None
	

	def device_follows_track(self, val):
		self._device_selection_follows_track_selection = (val == 1)
		return self
	

	def _update_selected_device(self):
		if self._device_selection_follows_track_selection is True:
			track = self.song().view.selected_track
			device_to_select = track.view.selected_device
			if device_to_select == None and len(track.devices) > 0:
				device_to_select = track.devices[0]
			if device_to_select != None:
				self.song().view.select_device(device_to_select)
			#self._device.set_device(device_to_select)
			self.set_appointed_device(device_to_select)
			#self._device_selector.set_enabled(True)
			self.request_rebuild_midi_map()
		return None 
	

	def assign_alternate_mappings(self, chan):
		#for column in range(8):
		#	for row in range(8):
		#		self._grid[column][row].set_identifier(OHM_MAP_ID[column][row])
		#		self._grid[column][row].set_channel(OHM_MAP_CHANNEL[column][row])
		#		self._grid[column][row].send_value(OHM_MAP_VALUE[column][row])
		#		self._grid[column][row].set_enabled(False)
		for index in range(8):
			self._encoder_button[index + 4].set_channel(chan)
			self._encoder_button[index + 4].set_enabled(chan is 0)
		for encoder in self._encoder:
			encoder.set_channel(chan)
			encoder.set_enabled(chan is 0)
		for button in self._button:
			button.set_channel(chan)
			button.set_enabled(chan is 0)
		for cell in self._grid:
			cell.set_channel(chan)
			cell.set_enabled(chan is 0)
		self.request_rebuild_midi_map()
			
	

	def get_session_offsets(self):
		if(self._is_split is True):
			return [self._session.track_offset(), self._session.scene_offset(), self._session2.track_offset(), self._session2.scene_offset()]
		elif(self._is_split is False):
			return [self._session_main.track_offset(), self._session_main.scene_offset(), (self._session_main.track_offset()) + 4, self._session_main.scene_offset()]
	

	def set_split_mixer(self, is_split):
		assert isinstance(is_split, type(False))
		if(is_split!=self._is_split):
			if(is_split is True):
				self._mixer._track_offset = self._session._track_offset
			else:
				self._mixer._track_offset = self._session_main._track_offset
			self._is_split = is_split
			self._session_main.set_enabled(not is_split)
			self._session.set_enabled(is_split)
			self._session2.set_enabled(is_split)
			self._mixer._reassign_tracks()
	

	def set_split_mixer_monomod(self, is_split):
		assert isinstance(is_split, type(False))
		if(is_split!=self._is_split):
			if(is_split is True):
				self._mixer._track_offset = self._session._track_offset
			else:
				self._mixer._track_offset = self._session_main._track_offset
			self._is_split = is_split
			self._mixer._reassign_tracks()
	

	def split_mixer(self):
		return self._is_split
	

	def _get_num_tracks(self):
		return self.num_tracks
	

	def _recalculate_selected_channel(self):
		selected = False
		for index in range(4):
			if self.song().view.selected_track == self._mixer.channel_strip(index)._track:
				selected = True
			elif self.song().view.selected_track == self._mixer2.channel_strip(index)._track:
				selected = True
		if selected is False:
			self.song().view.selected_track = self._mixer2.channel_strip(0)._track
	

	def mixer_on_cf_assign_changed(self, channel_strip):
		def _on_cf_assign_changed():
			if (channel_strip.is_enabled() and (channel_strip._crossfade_toggle != None)):
				if (channel_strip._track != None) and (channel_strip._track in (channel_strip.song().tracks + channel_strip.song().return_tracks)):
					if channel_strip._track.mixer_device.crossfade_assign == 1: #modified
						channel_strip._crossfade_toggle.turn_off()
					elif channel_strip._track.mixer_device.crossfade_assign == 0:
						channel_strip._crossfade_toggle.send_value(1)
					else:
						channel_strip._crossfade_toggle.send_value(2)
		return _on_cf_assign_changed
		
	

	def _device_update(self, device):
		def _update():
			DeviceComponent.update(device)
			self._update_selected_device()
		return _update
		

	def _on_session_offset_changes(self):
		if self._r_function_mode._mode_index in range(0,3):
			self._mem[int(self._r_function_mode._mode_index)] = self._session2.track_offset()
	
#	a
