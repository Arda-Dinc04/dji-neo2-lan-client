import { NestFactory } from "@nestjs/core";
import { AppModule } from "./app.module";

async function bootstrap() {
  const app = await NestFactory.create(AppModule, { rawBody: true });
  const webOrigin = process.env.WEB_ORIGIN || "http://localhost:5173";
  app.enableCors({ origin: webOrigin, credentials: true });
  app.setGlobalPrefix("home-server/api");
  const port = Number(process.env.PORT || 3000);
  await app.listen(port, "0.0.0.0");
  console.log(`Home-server API running on http://0.0.0.0:${port}`);
}
void bootstrap();
