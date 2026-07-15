// 校验 docs/demo.html 内联 <script> 的 JS 语法可解析，防止提交带语法错误的 demo。
// 用法：node scripts/check_demo_html.js
// 输出契约：成功打印 `demo_html=PASS inline_scripts=N` 退出 0；失败打印 FAIL 行退出 1。
const fs = require('fs');

const htmlPath = 'docs/demo.html';
try {
  const html = fs.readFileSync(htmlPath, 'utf8');
  const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map((m) => m[1]);
  scripts.forEach((script, index) => {
    try {
      // 只做语法解析，不执行脚本本体。
      new Function(script);
    } catch (err) {
      console.error(`demo_html=FAIL script_index=${index} error=${err.message}`);
      process.exit(1);
    }
  });
  console.log(`demo_html=PASS inline_scripts=${scripts.length}`);
} catch (err) {
  console.error(`demo_html=FAIL error=${err.message}`);
  process.exit(1);
}
