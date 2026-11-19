import importlib.util
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import threading
import unittest
from unittest import mock
import urllib.error
import urllib.request

spec=importlib.util.spec_from_file_location('panel_gateway',Path(__file__).resolve().parents[1]/'app/console_gateway.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.server=ThreadingHTTPServer(('127.0.0.1',0),m.Handler)
        self.port=self.server.server_port
        self.addCleanup(self.server.server_close)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.server.shutdown)
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(m,'PANEL_MODE',True).start()
        mock.patch.object(m,'PANEL_PORT',self.port).start()
        mock.patch.object(m.Handler,'log_message',lambda *a:None).start()
        self.proxy=mock.patch.object(m,'panel_proxy',return_value=(200,b'{"state":"unknown"}')).start()

    def request(self,path='/api/motors',body=None,method=None,origin=True):
        headers={'Content-Type':'application/json'}
        if origin:headers['Origin']='https://127.0.0.1:'+str(self.port)
        if isinstance(origin,str):headers['Origin']=origin
        request=urllib.request.Request('http://127.0.0.1:'+str(self.port)+path,data=body,headers=headers,method=method)
        try:
            with urllib.request.urlopen(request,timeout=2) as r:return r.status,r.read()
        except urllib.error.HTTPError as e:
            with e:return e.code,e.read()

    def test_get_only_reads_state(self):
        self.assertEqual(self.request()[0],200)
        self.proxy.assert_called_once_with('/state')

    def test_fixed_post_maps_only_to_matching_host_route(self):
        self.assertEqual(self.request(body=b'{"enabled":true}')[0],200)
        self.proxy.assert_called_once_with('/set',b'{"enabled":true}')
        self.proxy.reset_mock()
        self.assertEqual(self.request('/api/motors/initialize',b'{}')[0],200)
        self.proxy.assert_called_once_with('/initialize',b'{}')

    def test_cross_origin_missing_origin_size_and_verbs_do_not_dispatch(self):
        for origin in (False,'https://unrelated.invalid'):
            self.assertEqual(self.request(body=b'{}',origin=origin)[0],403)
        self.assertEqual(self.request(body=b'x'*513)[0],413)
        for verb in ('PUT','PATCH','DELETE'):
            self.assertEqual(self.request(body=b'{}',method=verb)[0],405)
        self.proxy.assert_not_called()

    def test_readonly_console_still_rejects_legacy_control(self):
        with mock.patch.object(m,'PANEL_MODE',False):
            self.assertEqual(self.request(body=b'{"enabled":true}')[0],409)
        self.proxy.assert_not_called()

    def test_panel_is_opt_in_and_original_motor_script_stays_removed(self):
        source='<html><head><script src="motor-toggle.js"></script></head></html>'
        plain=m.console_html(source)
        panel=m.console_html(source,True)
        self.assertNotIn('motor-toggle.js',plain)
        self.assertNotIn('test-motor-panel.js',plain)
        self.assertIn('test-motor-panel.js',panel)
        self.assertIn('__OPENARM_TEST_PANEL__=true',panel)


if __name__=='__main__':unittest.main()
