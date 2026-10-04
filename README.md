# langxin11 学习笔记发布入口

正式网站：<https://langxin11.github.io/>。

文章、图片、个人配置和 Shirone npm 依赖在 [内容仓库](https://github.com/langxin11/Mizuki_Content_XDL) 维护。本仓库的 `Publish Shirone notes` 工作流锁定内容提交，检查后部署到 GitHub Pages；内容仓库更新每小时检查，也可以手动运行。具体写作、立即发布与回退方式见内容仓库 `docs/正式发布与维护.md`。

迁移前的 Mizuki 源码仍保留作回退参考，不再作为当前网站构建入口。旧工作流已归档在 `.github/legacy-workflows/`；新的构建使用 Node.js 24 和内容仓库的锁文件，不再更新旧主题依赖。

Vercel 的构建入口也读取同一内容项目；后续只更新内容仓库时，需要在 Vercel 自行触发重部署。GitHub Pages 为主要发布入口。
