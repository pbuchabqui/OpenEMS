#pragma once
static const char kPageHtml[] =
"<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
"<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
"<title>RPM</title>"
"<style>"
"body{font-family:sans-serif;background:#111;color:#eee;max-width:400px;margin:0 auto;padding:20px}"
"h1{font-size:20px;text-align:center}"
".r{font-size:80px;text-align:center;font-weight:bold;color:#4fc3f7}"
"input[type=range]{width:100%;height:44px}"
"</style></head><body>"
"<h1>RPM</h1><div class=\"r\" id=\"v\">500</div>"
"<input type=\"range\" id=\"s\" min=\"50\" max=\"9000\" value=\"500\" step=\"10\" oninput=\"up(this.value)\">"
"<script>"
"var t=0;function up(x){v.textContent=x;clearTimeout(t);t=setTimeout(function(){fetch('/s?r='+x)},300)}"
"setInterval(function(){fetch('/s').then(r=>r.text()).then(t=>{"
"try{var j=JSON.parse(t);v.textContent=j.rpm;s.value=j.rpm}catch(e){}})},1000);"
"</script></body></html>";
